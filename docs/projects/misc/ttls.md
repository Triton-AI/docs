---
title: Target Trajectory Lines (TTLS)
hide_title: true
sidebar_pos: 1
---
import Template from "@site/static/img/projects/misc/outer_template.png";
import BoundaryTemplate from "@site/static/img/projects/misc/boundaries_template.png";
import RegionTemplate from "@site/static/img/projects/misc/regions_template.png";
import Track from "@site/static/img/projects/misc/track.png";
import Regions from "@site/static/img/projects/misc/regions.png";
import Reference from "@site/static/img/projects/misc/reference.png";

import ImageCard from "@site/src/components/ImageCard";
import ImageGallery from "@site/src/components/ImageGallery";

:::info
By the end of this document, you will understand:

- What a **Target Trajectory Line** is  
- How to **generate an optimal trajectory**
- How to **enforce custom behaviors** using defined regions
:::

## Requirements
- [ucsd-ttl-ws](https://github.com/Triton-AI/ucsd-ttl-ws) which includes all necessary workspace material
- [spline-trajectory-optimization](https://github.com/Triton-AI/spline-trajectory-optimization/tree/winston/ucsd-ttl) for creating an optimal TTL

## Target Trajectory Lines (TTLS)
A target trajectory line refers to the ideal trajectory that a vehicle should follow to reach its destination safely and efficiently. A path is is a series of points that goes from point A to B, but a trajectory has a time compoment and the velocity at each time stamp along the path.

## Google Earth
The beginning of TTLs starts with Google Earth. Please download the template KML file found [here](https://github.com/Triton-AI/ucsd-ttl-ws/blob/main/TEMPLATE/Triton-AI_Earth_WS_Template_temp.kml) and import it. After importing, you should have the following below.

<ImageGallery
  items={[
    {
      src: Template,
      alt: 'Template',
    },
    {
      src: BoundaryTemplate,
      alt: 'BoundaryTemplate',
    },
    {
      src: RegionTemplate,
      alt: 'RegionTemplate',
    }
  ]}
/>

You will notice there are two folders: boundaries and regions.

### Boundaries
Boundaries are your tracks. They are the inner and outer limits of the race track. Inside of boundary is the pit, track, and reference. We have separate boundaries for pit and track because there is a pit lane and a track lane. The reference point is simply a point in the area that acts as the origin for your coordinate frame. 

If you visualize mathematically, the origin will be used in the scripts later on as (0, 0, 0), and therefore, different origins will generate different TTLs.

:::danger
It is important that you do NOT enclose boundaries. Conventionally, we refer to polygons as regions and rest as boundaries. Regions must be enclosed.
:::

### Regions
Regions are enclosed areas where we can enforce specific behaviors in the codebase. That's all they are.

## Walkthrough
Using the template folders, let's proceed in creating our boudnaries and regions.

### Creating our Boundary
Please use the "Add path or polygon" icon and create the track. It is again important that it doesn't connect. Please ignore the example below as I made the mistake of connecting it. If you do connect, do not worry and go to the next section, as you can just delete the last point to "unconnect" it.

<ImageCard
    src={Track}
    alt="Track"
/>

### Creating our Region
Do the same for the regions but make sure they are closed. Ideally, if you want specific behavior at specific areas, you need to make sure they are a region. We only need the pit regions for now, and so in the example, ignore the straight and turns regions I have made.

<ImageCard
    src={Regions}
    alt="Region"
/>

### Adding an Origin
Add a placemark at put it somewhere on the map, ideally the starting point.

<ImageCard
    src={Reference}
    alt="Reference"
/>

After you finish everything, export it as a KML file.

## ucsd-ttl-ws
The ucsd-ttl-ws is a workspace for creating our TTLs. It contains all the necessary files, dependencies, scripts for conversions. Please go and clone [ucsd-ttl-ws](https://github.com/Triton-AI/ucsd-ttl-ws). Afterwards, please run the script ```run.sh``` to get into the Docker container.

Please create a new branch if you are creating new TTLs. For the time being, please create a branch using the pseudo ```<name>/learning-ttl```.

### Folder Structure
There is a folder structure we must adhere to and is shown below:

```bash
Triton-AI/ucsd-ttl-ws/P704/
├── enu/
│   ├── boundaries/
│   │   ├── pit/
│   │   │   ├── pit_inner_enu.csv
│   │   │   └── pit_outer_enu.csv
│   │   └── track/
│   │       ├── track_inner_enu.csv
│   │       └── track_outer_enu.csv
│   ├── references/
│   │   └── center.csv
│   └── regions/
│       ├── pit/
│       │   ├── pit_box/
│       │   │   └── pit_box_enu.csv
│       │   ├── pit_crawl/
│       │   │   ├── pit_crawl_enu.csv
│       │   │   ├── pit_crawl_in_enu.csv
│       │   │   └── pit_crawl_out_enu.csv
│       │   ├── pit_entry/
│       │   │   └── pit_entry_enu.csv
│       │   ├── pit_exit/
│       │   │   └── pit_exit_enu.csv
│       │   ├── pit_lane/
│       │   │   └── pit_lane_enu.csv
│       │   └── pit_road/
│       │       ├── pit_road_in_enu.csv
│       │       └── pit_road_out_enu.csv
│       └── track/
│           ├── straights/
│           │   ├── 0_straight/straight_0_enu.csv
│           │   ├── 1_straight/straight_1_enu.csv
│           │   ├── 2_straight/straight_2_enu.csv
│           │   ├── 3_straight/straight_3_enu.csv
│           │   ├── 4_straight/straight_4_enu.csv
│           │   ├── 5_straight/straight_5_enu.csv
│           │   ├── 6_straight/straight_6_enu.csv
│           │   ├── 7_straight/straight_7_enu.csv
│           │   └── 8_straight/straight_8_enu.csv
│           └── turns/
│               ├── 1_turn/turn_1_enu.csv
│               ├── 2_turn/turn_2_enu.csv
│               ├── 3_turn/turn_3_enu.csv
│               ├── 4_turn/turn_4_enu.csv
│               ├── 5_turn/turn_5_enu.csv
│               ├── 6_turn/turn_6_enu.csv
│               ├── 7_turn/turn_7_enu.csv
│               └── 8_turn/turn_8_enu.csv
├── gps/
│   ├── boundaries/
│   ├── reference_point/
│   │   └── origin.csv
│   └── regions/
└── ttls/
```
:::info
The boundaries and regions has the same structure but the gps files goes inside there. ```center.csv``` is a centerline, and will be shown after we convert KML to enu.
:::

### KML to ENU
Before proceeding, please make sure to create all the necessary folders. Before converting from KML to enu, please add the origin point into ```kml_extractor_custom.py``` by checking the Google Earth reference point. 

You can then run the command inside of your TTL folder by running ```python3 kml_extractor_custom.py input.kml``` and pressing ```ALL:```. This will convert all the KML into enu. If they are not organized, please put them into the folders.

### Centerline Reference
We can now create a general centerline reference that will be within the track bounds. If you have accidentally connected your tracks on Google Earth, you can remove the last line by manually editing it out in both inner and outer files.

Please use the file ```find_center_line_2.py``` found in the utils folder and select ```ALL``` at the bottom. The usage is as follows:

```bash
python find_center_line_2.py <outer_boundary_csv> <inner_boundary_csv> <output_csv>
```

Afterwards, please organize everything into their respective folders.

## spline-trajectory-optimization
Please clone the [spline-trajectory-optimization](https://github.com/Triton-AI/spline-trajectory-optimization/tree/winston/ucsd-ttl) repository Afterwards, please configure the yaml file according to the files available in either [P701](https://github.com/Triton-AI/ucsd-ttl-ws/tree/winston/p701-dev-0530) or [P704](https://github.com/Triton-AI/ucsd-ttl-ws/tree/kevin/p704) and head to the TTL folder, ```export TTL_DIR=$(pwd)```then run:

```bash
python3 traj_opt_double_track.py
```

## Visualizing
IF you wish to visualize, please use ```generate_gps_visualizers_zip.py```. Please note that you must add the origin point to the ttl and the csv file has to be within a folder as in ```P701/ttl.csv```. The origin point is added on the first line as there should be 6 but there is only 3. The last 3 is the origin point. Please make sure there are no spaces.

Then head over to [GPS Visualizer](https://www.gpsvisualizer.com/) and then export to Google Earth format and load it into the Google Earth.

## Offline Tools
TBD (For Regions)


