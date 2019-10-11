# RoadMarkingExtraction
A C++ Program for automatically extraction of road markings from MLS or ALS point cloud

![alt text](demo/MLS_demo.gif)

### Under Development (V_1.0.1)

## About
Compile with VS2013 x64 Release or Debug

Dependent 3rd Party Libs:  PCL 1.8, OpenCV 2 , LibLas, DXFLib

Application Scenarios: MLS，ALS point cloud , Highway and City roads

## Paper
Yue Pan, Bisheng Yang, Shengfu Li, Hong Yang, Zhen Dong, Xue Yang, Automatic Road Markings Extraction, Classification and Vectorization from Mobile Laser Scanning Data, ISPRS Geospatial Week 2019 (ISPRS Archives) [pdf](https://www.int-arch-photogramm-remote-sens-spatial-inf-sci.net/XLII-2-W13/1089/2019/)

## How to use
1.Import a las file

2.Select datatype, roadtype and related information

3.Input the approximate point density ( number per m^2 )

4.Output all the classified road marking point clouds and the vectorization DXF file.

## Workflow
 ![alt text](demo/Fig1.jpg)

## Demo
 ![alt text](demo/Fig2.jpg)
 
 ![alt text](demo/Fig3.jpg)
 
