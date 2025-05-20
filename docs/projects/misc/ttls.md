---
title: Target Trajectory Lines (TTLS)
hide_title: true
sidebar_pos: 1
---

import Icon from "@site/static/img/projects/misc/icon.jpg";
import Outer from "@site/static/img/projects/misc/outer_ttls.png";
import Inner from "@site/static/img/projects/misc/inner_ttls.png";

import ImageCard from "@site/src/components/ImageCard";


## Target Trajectory Lines (TTLS)
A target trajectory line refers to teh ideal path that a vehicle should follow to reach its destination safely and efficiently. It's typically generated from a series of GPS waypoints or predefined goals and is the primary input for the vehicle's control system, assuming a simple waypoint follower.

GPS waypoints provide the basic framework, and is then interpolated int o a smooth, continuous curve, often using splines or polynomials to form the target trajectory line. Controllers like Pure Pursit, Stanley, or Model Predictive Control uses the current position of the vehicle and calculate the necessary control commands to minimize the deviation between the actual path and the target trajectory. 

## Generating KMLs
To generate TTLS, you can first generate the predefined path using [Google Earth](httpsL//earth.google.com/web/). These are saved as KML format. You need to head over to where you want the path on Earth. For reference, we'll use the JSOE EBU1 building for demo. To create a polygonal shape, press on the "Add path or polygon" icon.

<img src={Icon} />

:::info
Please have each boundary in separate Google Earth Projects
:::

### Outer
TTLs need an outer boundary that specifies the walls of the path. As such, we need to create a polygonal enclosing enclosing the race track. Below, you'll see an outer boundary for the EBU1 building.

<ImageCard
    src={Outer}
    alt="Outer TTLs"
    description="The Outer TTL boundary for JSOE EBU1"
/>

### Inner
You now need an inner boundary for the race track. Follow the exact same steps to get the below:

<ImageCard
    src={Inner}
    alt="Inner TTLs"
    description="The Inner TTL boundary for JSOE EBU1s"
/>

Export both the outer and inner as KML formats. 

### KML to CSV
Our TTL generator requires a CSV format, and Google Earth exports in KML. Assuming you have race_common on your computer, go into ```race_common/tools/kml_extractor``` and run:

```
python3 kml_extractor_polygon.py <KML_FILE>
```

## Generating TTLs
To generate TTLS, you need to clone this [repository](https://github.com/airacingtech/offline-trajectory-tools). 

