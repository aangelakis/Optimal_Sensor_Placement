# 💧 Optimal Sensor Placement in Water Distribution Networks (WDNs)

This repository implements algorithms to optimally place sensors in water distribution networks (WDNs) to detect contamination or leakage events efficiently. It combines spatial network analysis, graph theory, and domain-specific heuristics to minimize sensor count while maximizing coverage.

Two core algorithms are presented:
- The **base algorithm** ensures full network coverage by strategically positioning sensors within a defined range.
- The **extended algorithm** enhances reliability by placing additional sensors so that every edge in the graph is monitored by at least two sensors. This redundancy is particularly useful for detecting potential leaks in water pipes.

Detailed descriptions and demonstrations of the algorithms are provided in the accompanying Jupyter notebook.

---

## 📌 Features

- 🔍 **Sensor Coverage Algorithm**: Ensures maximum edge coverage within a defined sensor range.
- 🚨 **Leakage Detection Algorithm**: Identifies and resolves possible undetected leak paths.
- 🌐 **Geospatial Network Support**: Converts shapefiles of real-world networks into graph structures.
- 📊 **Visualization Tools**: Plots sensor placement and coverage on the network.

---

## 💠 Installation

1. **Clone the repo**
```bash
git clone https://github.com/aangelakis/Optimal_Sensor_Placement.git
cd Optimal_Sensor_Placement
```

2. **Install dependencies**
```bash
pip install -r requirements.txt
```

> Required Python version: **3.9+**

---

## 📂 Directory Structure

```
.
├── network/                      # Folder containing shapefiles (e.g., diktio.shp)
├── utils/                        # Folder with all helper modules and core logic
├── notebooks/                    # Interactive notebooks
├── code/                         # Python code
├── requirements.txt              # Python dependencies
└── README.md
```

---

## ▶️ How to Use

1. Replace `network/diktio.shp` with your own WDN shapefile (supported shapes: LineString, MultiLineString, Polygon, MultiPolygon).
2. Run:
```bash
python optimal_sensor_placement_in_WDNs.py
```
3. Follow the instructions inside the code:
   - Set `sensor_range`
   - Load or visualize your graph
   - Run coverage and leakage detection algorithms

---

## 📈 Visualization

- After execution, plots of the graph will show:
  - Sensor positions (highlighted)
  - Covered and uncovered edges
  - Coverage before and after leakage correction

> Requires `matplotlib` and `networkx` for visualization.

---

## 🧐 Algorithm Overview

- **Sensor Coverage**:
  - Performs a BFS to determine minimal sensor placements needed to cover the entire network within the defined range.
  
- **Leakage Detection**:
  - Identifies paths not covered due to topology limitations or edge cases.
  - Augments sensor placement to handle these edge cases.
  
- **Redundancy**:
  - Optional mode ensures every edge is covered by at least two sensors to enhance reliability and support leak localization.

---

## 🧪 Example Dataset

The default shapefile (`diktio.shp`) represents the water network of **Mohos village** in **Hersonissos, Crete, Greece**.

---

## 🤝 Contributing

Contributions are welcome! Please:

- Fork the repo
- Create a feature branch
- Submit a pull request

---

## 📄 License

MIT License © [Alexandros Angelakis](https://www.linkedin.com/in/alexandrosangelakis/)

---

## ✉️ Contact

For questions or collaboration, feel free to open an issue or contact via GitHub.
