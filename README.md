<table width="100%">
    <tr>
        <td width="25%">
            <img src="https://www.unige.ch/medecine/chiru/files/cache/4be03300652cdfa4cac6972e37e8665f_f99.png" alt="Biomechanics Laboratory (B-LAB) - University of Geneva" width="100%"/>
        </td>
        <td width="75%">
            <h1>Welcome on the Roboshoulder_toolbox repository</h1><br>
            <p>Thank you for your interest in our project. You can find further information about our research activites at the University of Geneva on our website: <a href="https://www.unige.ch/medecine/chiru/fr/b-lab-tests-robotises-avances-de-dispositifs-chirurgicaux/" target="_blank">https://www.unige.ch/medecine/chiru/fr/b-lab-tests-robotises-avances-de-dispositifs-chirurgicaux/</a>.</p>
            <p>The projects available on this repository are all freely available and opensource, under the license <a href="https://creativecommons.org/licenses/by-nc/4.0/" target="_blank">Creative Commons Attribution-NonCommercial 4.0 (International License)</a>.</p>
        </td>
    </tr>
</table>
<h2 align="left">PROJECT DESCRIPTION</h2>
This is the version 1 of the BLAB_Roboshoulder_toolbox (an archive is available on Zenodo (https://zenodo.org/badge/latestdoi/666377581). This toolbox is related to a dataset freely available on Zenodo (https://www.doi.org/10.5281/zenodo.8146967). The toolbox and dataset were established during the RoboShoulder project, a joined project with the <a href="https://www.hesge.ch/hepia/" target="_blank">HEPIA</a> school at Geneva, funded by the Orthopedic Surgery and Musculoskeletal Trauma Care Division of the Surgery Department of the Geneva University Hospitals.

</h2>

## Important Notices
* `master` branch file paths (if exist) are **not** considered stable.

## Table of Contents
[**Installation**](#installation)

[**Features**](#features)

[**Dependencies**](#dependencies)

[**Developer**](#developer)

[**Examples**](#examples)

[**References**](#references)

[**License**](#license)

## Installation
You just need to download or clone the project to use it. It has only been tested on Matlab R2022b (The Mathworks, USA).

## Features
- Load segmentation and motion data from the related dataset
- Process motion data
- Compute 6-DoF joint kinematics for every joint of the shoulder girdle
- Generate result tables and related figures

## Dependencies
- 3D Kinematics and Inverse Dynamics toolbox by Raphaël Dumas: https://fr.mathworks.com/matlabcentral/fileexchange/58021-3d-kinematics-and-inverse-dynamics
- Biomechanical ToolKit: https://github.com/Biomechanical-ToolKit
- PredictMissingMarkers: https://doi.org/10.1371/journal.pone.0152616   
- soder: https://doi.org/10.1016/0021-9290(93)90098-Y
- LSGE: The Least Squares Geometric Elements library: http://www.eurometros.org/gen_report.php?category=distributions&pkey=14

## Developer
The proposed routine has been developed by Florent Moissenet (PhD), B-LAB, Geneva University Hospitals and University of Geneva (florent.moissenet[at]unige.ch).

## Examples
Users are invited to download the freely available dataset related to this toolbox on Zenodo (10.5281/zenodo.8146967).

## References
Validation of a Robotic Testing Procedure for Shoulder In-Vitro Biomechanical Testing. World journal of surgery and surgical research. 2022;5(1):1392

## License
<a href="https://creativecommons.org/licenses/by-nc/4.0/legalcode" target="_blank">LICENSE</a> © B-LAB, Geneva University Hospitals and University of Geneva
