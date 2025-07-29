import streamlit as st
import subprocess

st.set_page_config(page_title="Drone2You Simulator", layout="centered")

st.title("🚁 Drone2You Delivery Simulation")

st.markdown("""
This simulator demonstrates a real-time drone dispatch system with surge pricing.

Click below to launch the full animated simulation in a separate window.
""")

if st.button("Launch Drone2You Simulation"):
    subprocess.Popen(["python", "fcfs.py"])
    st.success("Simulation window launched!")