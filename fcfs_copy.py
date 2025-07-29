import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import osmnx as ox
import contextily as ctx
from matplotlib.animation import FuncAnimation
from matplotlib.colors import to_hex
from queue import Queue, Empty
import threading
import time
import random
from collections import defaultdict
import streamlit as st
st.set_page_config(page_title="Drone2You Simulator", layout="wide")


# Parameters
total_drones = 30
timesteps = 300
base_fare = 100
dt = 1.0
order_timeout = 30
recharge_time = 10

# Surge Pricing Controller
tau, m_max, p = 5.0, 4.0, 1.2
m = np.ones(timesteps)
pi_1 = np.zeros(timesteps)
pi_1[0] = base_fare

# Demand (sinusoidal + noise)
t = np.arange(timesteps)
demand_series = 15 + 20 * np.sin(2 * np.pi * t / 60) + 5 * np.random.randn(timesteps)
demand_series = np.clip(demand_series, 0, None).astype(int)
matched_queue = demand_series.copy()  # for simplicity

# Map Initialization
#hq_coords = (-33.8688, 151.2093)  # Sydney CBD, Australia
hq_coords = (-37.8106, 144.9631) # Melbourne CBD, Australia
G = ox.graph_from_point(hq_coords, dist=1000, network_type='drive')
nodes = list(G.nodes)
hq_node = ox.distance.nearest_nodes(G, hq_coords[1], hq_coords[0])

# Drone State
drone_paths = {i: [] for i in range(total_drones)}
drone_indices = {i: 0 for i in range(total_drones)}
drone_active = {i: False for i in range(total_drones)}
drone_busy_until = {i: 0 for i in range(total_drones)}
drone_condition = {i: True for i in range(total_drones)}
drone_labels = {i: "" for i in range(total_drones)}
deployment_counts = {i: 0 for i in range(total_drones)}
drone_colors = [to_hex(plt.colormaps['tab10'](i % 10)) for i in range(total_drones)]
drone_batch_assignment = {i: None for i in range(total_drones)}
drone_dispatch_price = {}  # key = drone_id, value = π₁ at dispatch time

# Order Queues
pending_orders = []
pending_lock = threading.Lock()
order_timestamps = {}
restaurant_points = []
destination_points = {}
orders_generated = np.zeros(timesteps, dtype=int)

# Batch Tracking
batches = []
batch_lock = threading.Lock()
current_frame = 0
batch_id_counter = 0
drones_dispatched = np.zeros(timesteps, dtype=int)
dispatched_ids_log = [""] * timesteps
batch_dispatch_prices = defaultdict(list)  # batch_id -> list of dispatched π₁ values


# Logging
def log_event(msg): print(f"[{time.strftime('%H:%M:%S')}] {msg}")

def start_new_batch(matched, frame):
    global batch_id_counter
    with batch_lock:
        batch = {
            "batch_id": batch_id_counter,
            "order_count": matched,
            "drones_assigned": 0,
            "drones_completed": 0,
            "drones_pending": matched,
            "available_drones": [i for i in range(total_drones) if not drone_active[i] and drone_condition[i]],
            "error_at_batch": 0,
            #"price_at_batch": pi_1[frame],
            "profit": 0
        }
        batches.append(batch)
        #log_event(f"📦 Batch {batch_id_counter} created with {matched} orders. π₁ = RM {pi_1[current_frame]:.2f}")
        batch_id_counter += 1

# Order Generator
def generate_order_controlled():
    order_id = 1
    for k in range(timesteps):
        matched = matched_queue[k]
        orders_generated[k] = matched
        if matched > 0:
            start_new_batch(matched, k)
        created = 0
        while created < matched:
            if not any(drone_condition.values()):
                log_event("❌ All drones failed. Shutting down.")
                return
            rest, cust = np.random.choice(nodes, 2, replace=False)
            if rest != cust:
                if (nx.has_path(G, hq_node, rest) and nx.has_path(G, rest, cust) and nx.has_path(G, cust, hq_node)):
                    with pending_lock:
                        pending_orders.append((order_id, rest, cust))
                    order_timestamps[order_id] = time.time()
                    restaurant_points.append(rest)
                    order_id += 1
                    created += 1
        time.sleep(10)

# Drone Dispatcher
def assign_orders():
    while True:
        expired_orders = [oid for oid, t in order_timestamps.items() if time.time() - t > order_timeout]
        for oid in expired_orders:
            del order_timestamps[oid]
        for i in range(total_drones):
            if not drone_active[i] and drone_busy_until[i] <= time.time() and drone_condition[i]:
                try:
                    with pending_lock:
                        if pending_orders:
                            order_id, rest, dest = pending_orders.pop(0)
                        else:
                            continue
                    order_timestamps.pop(order_id, None)
                    with batch_lock:
                        for batch in batches:
                            if batch['drones_assigned'] < batch['order_count']:
                                target_batch = batch
                                break
                        else:
                            continue  # No pending batches, skip this loop

                        target_batch['drones_assigned'] += 1
                        drone_batch_assignment[i] = target_batch['batch_id']

                    if current_frame < timesteps:
                        drones_dispatched[current_frame] += 1
                        dispatched_ids_log[current_frame] += f"Drone {i}, "
                        pi_value = pi_1[current_frame]
                        drone_dispatch_price[i] = pi_value  
                        batch_dispatch_prices[target_batch['batch_id']].append(pi_value)
                        
                    path = nx.shortest_path(G, hq_node, rest) + \
                           nx.shortest_path(G, rest, dest)[1:] + \
                           nx.shortest_path(G, dest, hq_node)[1:]
                    drone_paths[i] = path
                    drone_indices[i] = 0
                    drone_active[i] = True
                    drone_labels[i] = f"Order {order_id}"
                    destination_points[i] = dest
                    deployment_counts[i] += 1
                except Empty:
                    continue
                except nx.NetworkXNoPath:
                    continue
        time.sleep(1)

# Batch Status Display
def display_batch_status():
    while True:
        time.sleep(2)
        with batch_lock:
            print("\n{:<8} {:<8} {:<12} {:<12} {:<12} {:<12} {:<10} {:<10}".format(
                "Batch", "Orders", "Dispatched", "Completed", "Pending", "Available", "Avg π₁", "Profit"))
            print("-" * 90)
            for batch in batches:
                available_count = len(batch['available_drones'])

                # Get average π₁ for this batch
                dispatch_prices = batch_dispatch_prices.get(batch['batch_id'], [])
                avg_pi = sum(dispatch_prices) / len(dispatch_prices) if dispatch_prices else 0

                print("{:<8} {:<8} {:<12} {:<12} {:<12} {:<12} RM{:<8.2f} RM{:<8.2f}".format(
                    batch['batch_id'], batch['order_count'], batch['drones_assigned'],
                    batch['drones_completed'], batch['drones_pending'], available_count,
                    avg_pi, batch['profit']))

# Surge Pricing Update
def update_price_model(frame):
    if frame == 0: return
    supply = max(1, sum(drone_condition[i] and drone_busy_until[i] <= time.time() for i in range(total_drones)))
    demand = max(1, demand_series[frame])
    ratio = demand / supply
    raw_m = ratio ** p
    target_m = np.clip(raw_m, 1.0, m_max)
    m[frame] = m[frame - 1] + (dt / tau) * (target_m - m[frame - 1])
    pi_1[frame] = base_fare * m[frame]

# Map + Visualization
fig = plt.figure(figsize=(15, 10))
gs = fig.add_gridspec(2, 2)
ax_map = fig.add_subplot(gs[:, 0])         # Large map on left
ax_price = fig.add_subplot(gs[0, 1])       # Top-right: price chart
ax_text = fig.add_subplot(gs[1, 1])        # Bottom-right: dashboard

ax_text.axis('off')
ax_text.set_title("📦 Batch Summary")
batch_text = ax_text.text(0, 1, "", fontsize=9, va='top', family='monospace')

edges = ox.graph_to_gdfs(G, nodes=False)
edges.plot(ax=ax_map, linewidth=0.5, edgecolor='gray')
ctx.add_basemap(ax_map, crs=edges.crs)
ax_map.set_title("Drone2You Map")
hq_x, hq_y = G.nodes[hq_node]['x'], G.nodes[hq_node]['y']
ax_map.plot(hq_x, hq_y, 's', color='black', markersize=10, label='HQ')
ax_map.legend()

dots = [ax_map.plot([], [], 'o', color=drone_colors[i])[0] for i in range(total_drones)]
# Restaurant markers (static)
restaurant_scatter = ax_map.scatter([], [], marker='^', c='orange', s=40, label="Restaurants")

# Destination markers (dynamic, colored per drone)
destination_scatters = {i: ax_map.scatter([], [], marker='X', c=drone_colors[i], s=40) for i in range(total_drones)}

# Drone trails
drone_trails = [ax_map.plot([], [], '-', color=drone_colors[i], linewidth=0.8)[0] for i in range(total_drones)]
drone_trail_coords = {i: [[], []] for i in range(total_drones)}

# Drone labels (ID + Order)
drone_texts = [ax_map.text(0, 0, '', fontsize=6, ha='center') for _ in range(total_drones)]

# Store restaurant and destination node coordinates
restaurant_coords = {'x': [], 'y': []}
destination_coords = {i: {'x': [], 'y': []} for i in range(total_drones)}

price_line, = ax_price.plot([], [], color='green', label='π₁ (RM)')
ax_price.set_title("Surge Price π₁")
ax_price.set_xlim(0, timesteps)
ax_price.set_ylim(0, 500)
ax_price.grid(True)
ax_price.legend()

# Animation Update
def update(frame):
    global current_frame
    current_frame = frame
    update_price_model(frame)
    with pending_lock:
        pending_count = len(pending_orders)
    threshold = 10
    title_color = 'red' if pending_count > threshold else 'black'
    ax_map.set_title(f"Drone2You Map | Pending Orders: {pending_count}", color=title_color)

    for i in range(total_drones):
        if drone_active[i] and drone_indices[i] < len(drone_paths[i]):
            node = drone_paths[i][drone_indices[i]]
            x, y = G.nodes[node]['x'], G.nodes[node]['y']
            dots[i].set_data([x], [y])

            # Trail
            drone_trail_coords[i][0].append(x)
            drone_trail_coords[i][1].append(y)
            drone_trails[i].set_data(drone_trail_coords[i][0], drone_trail_coords[i][1])

            # Label
            drone_texts[i].set_position((x, y + 5))
            drone_texts[i].set_text(f"ID:{i}\n{drone_labels[i]}")

            drone_indices[i] += 1

        elif drone_active[i] and drone_indices[i] >= len(drone_paths[i]):
            drone_active[i] = False
            drone_busy_until[i] = time.time() + recharge_time
            batch_id = drone_batch_assignment.get(i)
            if batch_id is not None:
                with batch_lock:
                    for batch in batches:
                        if batch['batch_id'] == batch_id:
                            batch['drones_completed'] += 1
                            batch['drones_pending'] -= 1
                            price = drone_dispatch_price.get(i, 0)
                            batch['profit'] += price

        # Restaurants (once)
    if restaurant_coords['x'] == [] and len(restaurant_points) > 0:
        restaurant_coords['x'] = [G.nodes[n]['x'] for n in restaurant_points]
        restaurant_coords['y'] = [G.nodes[n]['y'] for n in restaurant_points]
        restaurant_scatter.set_offsets(np.c_[restaurant_coords['x'], restaurant_coords['y']])

    # Destinations (update per drone)
    for i in range(total_drones):
        dest = destination_points.get(i)
        if dest:
            x, y = G.nodes[dest]['x'], G.nodes[dest]['y']
            destination_coords[i]['x'] = [x]
            destination_coords[i]['y'] = [y]
            destination_scatters[i].set_offsets(np.c_[destination_coords[i]['x'], destination_coords[i]['y']])

    price_line.set_data(np.arange(frame+1), pi_1[:frame+1])

    # Build live dashboard text
    summary_lines = ["{:<6} {:<8} {:<10} {:<10} {:<10} {:<10} {:<8} {:<8}".format(
        "ID", "Orders", "Dispatched", "Completed", "Pending", "Avail", "Avg π₁", "Profit")]

    with batch_lock:
        for b in batches[-20:]:
            dispatch_prices = batch_dispatch_prices.get(b['batch_id'], [])
            avg_pi = sum(dispatch_prices) / len(dispatch_prices) if dispatch_prices else 0
            summary_lines.append("{:<6} {:<8} {:<10} {:<10} {:<10} {:<10} RM{:<6.2f} RM{:<6.2f}".format(
                b['batch_id'], b['order_count'], b['drones_assigned'],
                b['drones_completed'], b['drones_pending'],
                len(b['available_drones']), avg_pi, b['profit']))

    batch_text.set_text("\n".join(summary_lines))

    return dots + [price_line, batch_text] + list(drone_trails) + drone_texts + list(destination_scatters.values()) + [restaurant_scatter]

if st.button("▶️ Start Simulation"):
    # Launch Threads
    threading.Thread(target=generate_order_controlled, daemon=True).start()
    threading.Thread(target=assign_orders, daemon=True).start()
    #threading.Thread(target=display_batch_status, daemon=True).start()
    ani = FuncAnimation(fig, update, frames=timesteps, interval=300, blit=False)
    st.title("🚁 Drone2You Delivery Simulation")
    st.markdown("This real-time drone dispatch simulation visualizes surge pricing, drone paths, and live status.")
    st.pyplot(fig)
