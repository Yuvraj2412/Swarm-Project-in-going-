# Generate a more detailed "mini research proposal" style PDF with sections, pseudocode, and simple diagrams
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, Preformatted
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm

file_path = "/mnt/data/UAV_Myopic_Coverage_Path_Planning_Extension_Document.pdf"

styles = getSampleStyleSheet()
mono = ParagraphStyle('mono', parent=styles['BodyText'], fontName='Courier')

title = Paragraph(
"Extension of Myopic Coverage Path Planning from Rover to UAV\n(Technical Summary & Research Proposal Outline)",
styles['Title']
)

intro = Paragraph("""
This document summarizes the engineering modifications required to adapt the algorithm presented in
“Risk-Aware Coverage Path Planning for Lunar Micro-Rovers Leveraging Global and Local Environmental Data”
to an Unmanned Aerial Vehicle (UAV). The goal is to maintain the lightweight myopic coverage strategy while
extending the system to operate in a 3D exploration environment with aerial vehicle dynamics.
""", styles['BodyText'])

section1 = Paragraph("<b>1. Key Structural Changes</b>", styles['Heading2'])

table_data = [
["Component","Original Rover System","Proposed UAV System"],
["State Space","2D grid (x,y)","3D voxel grid (x,y,z)"],
["Neighbor Set","8 neighboring cells","26 neighboring voxels"],
["Terrain Risk","DEM slope cost","Altitude change + obstacle clearance risk"],
["Energy Model","Wheel rolling energy","Thrust-based flight energy model"],
["Obstacle Avoidance","Bug algorithm","Artificial Potential Field (APF)"],
["SLAM","HDL Graph SLAM","LIO-SAM (LiDAR + IMU)"],
["Map Representation","2D occupancy grid","3D voxel map"]
]

table = Table(table_data, colWidths=[5*cm,6*cm,6*cm])

section2 = Paragraph("<b>2. UAV Mathematical Models</b>", styles['Heading2'])

math_text = """
Altitude Risk Cost

mc_alt = w1|z(t+1) − z(t)| + w2(1 / d_obs) + w3|z(t+1) − z_ref|

Energy Consumption Model

E_move = P_hover * t + m*g*(z(t+1) − z(t)) + F_drag * d

Hover Power Model (Momentum Theory)

P_hover = (m*g)^(3/2) / sqrt(2 * rho * A)

Final UAV Cost Function

mc = α(mc_static + mc_visited * V_i) + β mc_alt + γ mc_energy
"""

math_block = Preformatted(math_text, mono)

section3 = Paragraph("<b>3. Algorithm Flow</b>", styles['Heading2'])

flow_text = """
UAV Sensors (LiDAR + IMU)
        ↓
LIO-SAM SLAM
        ↓
3D Voxel Map
        ↓
Myopic Coverage Planner
        ↓
Compute Cost for 26 Neighbor Voxels
(static + visited + altitude risk + energy)
        ↓
Artificial Potential Field Obstacle Avoidance
        ↓
Select Minimum Cost Voxel
        ↓
UAV Motion Controller
"""

flow_block = Preformatted(flow_text, mono)

section4 = Paragraph("<b>4. UAV Coverage Algorithm Pseudocode</b>", styles['Heading2'])

pseudo = """
Initialize 3D voxel map
Initialize UAV position

while coverage < target:

    sense nearby voxels
    update occupancy map

    neighbors = generate 26 neighbors

    for each neighbor:
        compute motion cost
        cost = static + visited + altitude + energy

    choose neighbor with minimum cost

    apply APF obstacle avoidance
    move UAV to selected voxel

    update SLAM pose

end
"""

pseudo_block = Preformatted(pseudo, mono)

section5 = Paragraph("<b>5. Expected Research Contributions</b>", styles['Heading2'])

contrib = Paragraph("""
• Extension of myopic coverage path planning to 3D UAV exploration environments.<br/>
• Introduction of altitude-aware risk cost functions for aerial navigation.<br/>
• Integration of energy-aware motion planning based on UAV thrust models.<br/>
• Development of a lightweight 3D coverage planner compatible with real-time UAV systems.<br/>
• Implementation in simulation environments such as MATLAB UAV Toolbox, Gazebo, or CoppeliaSim.
""", styles['BodyText'])

story = [
    title, Spacer(1,20),
    intro, Spacer(1,20),
    section1, Spacer(1,10),
    table, Spacer(1,20),
    section2, Spacer(1,10),
    math_block, Spacer(1,20),
    section3, Spacer(1,10),
    flow_block, Spacer(1,20),
    section4, Spacer(1,10),
    pseudo_block, Spacer(1,20),
    section5, Spacer(1,10),
    contrib
]

doc = SimpleDocTemplate(file_path, pagesize=A4)
doc.build(story)

file_path