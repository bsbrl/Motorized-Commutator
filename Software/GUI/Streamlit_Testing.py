import streamlit as st
import streamlit.components.v1 as components


st.set_page_config(
    page_title="Commutator", page_icon="🍥", initial_sidebar_state="collapsed"
)


def change_label_style(label, font_size='20px', font_color='white', font_family='sans-serif'):
    html = f"""
    <script>
        var elems = window.parent.document.querySelectorAll('p');
        var elem = Array.from(elems).find(x => x.innerText == '{label}');
        elem.style.fontSize = '{font_size}';
        elem.style.color = '{font_color}';
        elem.style.fontFamily = '{font_family}';
    </script>
    """
    components.html(html)


# Function to load and apply CSS
with open("style.css") as f:
    st.markdown(f'<style>{f.read()}</style>', unsafe_allow_html=True)


st.markdown("# Motorized Commutator")

st.write("")

dlc_container = st.container(border=True)

dlc_container.write("### DLC Config")

dlc_container_col1, dlc_container_col2 = dlc_container.columns([2, 1])

select_model_label = "Select DLC Model"
dlc_Select = dlc_container_col1.text_input(
    select_model_label,
    key="dlc_model",
)
change_label_style(select_model_label)

dlc_browse_label = "Browse"
dlc_container_col2.button(
    dlc_browse_label,
    key="dlc_browse",
    type="primary"
)

display_tracking_label = "Display Tracking"
dlc_container.checkbox(display_tracking_label, key="display")
change_label_style(display_tracking_label)

# dlc_Select = dlc_container.text_input(
#     r"$\textsf{\normalsize Select DLC Model}$",
#     key="address",
# )

# dlc_container.checkbox(r"$\textsf{\normalsize Display Tracking}$", key="display")

# col1, col2 = container.columns([2, 1])

# address = col1.text_input(
#     "Select DLC Model",
#     key="address",
# )

# radius = col2.slider(
#     "Display Tracking",
#     100,
#     1500,
#     key="radius",
# )

# style: str = col3.selectbox(
#     "Color theme",
#     options=list(),
#     key="style",
# )
