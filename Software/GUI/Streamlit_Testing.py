import streamlit as st

st.set_page_config(
    page_title="Commutator", page_icon="🍥", initial_sidebar_state="collapsed"
)

st.markdown("# Motorized Commutator")

st.write("")

container = st.container(border=True)

container.write("### DLC Config")

col1, col2 = container.columns([2, 1])

address = col1.text_input(
    "Select DLC Model",
    key="address",
)

radius = col2.slider(
    "Display Tracking",
    100,
    1500,
    key="radius",
)

# style: str = col3.selectbox(
#     "Color theme",
#     options=list(),
#     key="style",
# )
