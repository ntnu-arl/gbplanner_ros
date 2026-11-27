#!/usr/bin/env python3
"""Compare selected organized lidar rows with static world mesh intersections.

Reads actual PointCloud2 bytes and acquisition-time raw mapper transforms.
Reuses the two-sided float64 triangle query in analyze_scan_endpoints.py.
No scan is integrated and no robot or production configuration is changed.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from analyze_scan_endpoints import DEFAULT_MESH, TriangleRays, read_mesh
from compare_scan_pose import quaternion, rotate


def read_cloud(snapshot, path):
    cloud = snapshot["cloud"]
    width, height = cloud["width"], cloud["height"]
    stride, row_stride = cloud["point_step"], cloud["row_step"]
    fields = {field["name"]: field for field in cloud["fields"]}
    data_file = path.parent / cloud["data_file"]
    data = data_file.read_bytes()
    if len(data) != cloud["bytes"] or len(data) != row_stride * height or row_stride < width*stride:
        raise ValueError("Cloud byte layout differs from metadata")
    dtype = ">f4" if cloud["is_bigendian"] else "<f4"
    xyz = []
    for axis in "xyz":
        field = fields[axis]
        if field["datatype"] != 7 or field["count"] != 1 or field["offset"] + 4 > stride:
            raise ValueError("Expected scalar float32 XYZ")
        xyz.append(np.ndarray((height, width), dtype=dtype, buffer=data,
                              offset=field["offset"], strides=(row_stride, stride)).astype(np.float64))
    return np.stack(xyz, axis=-1), data_file, hashlib.sha256(data).hexdigest()


def distribution(values):
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if not len(values):
        return {"count": 0}
    return {"count": len(values), "min": float(np.min(values)), "max": float(np.max(values)),
            "mean": float(np.mean(values)), "stddev": float(np.std(values)),
            "median": float(np.median(values)), "p05": float(np.quantile(values, .05)),
            "p95": float(np.quantile(values, .95)), "rms": float(np.sqrt(np.mean(values**2)))}


def analyze(path, mesh, rows, column_stride):
    data = path.read_bytes()
    snapshot = json.loads(data)
    points, cloud_path, cloud_hash = read_cloud(snapshot, path)
    transform = snapshot["raw_transform"]
    origin = np.asarray(transform["origin"], dtype=np.float64)
    rotation = quaternion(transform["rotation_wxyz"])
    records = []
    for row in rows:
        if not 0 <= row < points.shape[0]:
            raise ValueError("Requested row exceeds captured height")
        for column in range(0, points.shape[1], column_stride):
            point = points[row, column]
            if not np.isfinite(point).all():
                continue
            measured = float(np.linalg.norm(point))
            if measured <= 0.0:
                continue
            elevation = math.degrees(math.atan2(point[2], np.linalg.norm(point[:2])))
            if elevation >= 0.0:
                continue
            direction = rotate(rotation, point / measured)
            direction /= np.linalg.norm(direction)
            endpoint = origin + measured*direction
            hit = mesh.first_hit(origin, direction)
            record = {"row": row, "column": column, "point_index": row*points.shape[1]+column,
                      "sensor_azimuth_deg": math.degrees(math.atan2(point[1], point[0])),
                      "sensor_elevation_deg": elevation, "measured_range_m": measured,
                      "point_sensor": point.tolist(), "endpoint_world": endpoint.tolist(),
                      "mesh_hit": hit, "range_residual_m": None, "planar_ground_hit": False}
            if hit is not None:
                record["range_residual_m"] = measured - hit["range_m"]
                normal = np.asarray(hit["normal_world"])
                record["planar_ground_hit"] = bool(abs(normal[2]) > .99)
                record["endpoint_height_above_intersected_plane_m"] = float(np.dot(normal, endpoint-np.asarray(hit["point_world"])))
                plane_distance = abs(float(np.dot(normal, origin-np.asarray(hit["point_world"]))))
                incidence = abs(float(np.dot(normal, direction)))
                # This is the angle a same-range beam would need to hit this
                # plane, not a claim that the true beam used this direction.
                implied = plane_distance / measured
                record["plane_implied_incidence_angle_difference_deg"] = (
                    math.degrees(math.asin(implied)-math.asin(min(1.0, incidence))) if implied <= 1.0 else None)
            records.append(record)
    by_row = []
    for row in rows:
        selected = [r for r in records if r["row"] == row]
        ground = [r for r in selected if r["planar_ground_hit"]]
        residuals = [r["range_residual_m"] for r in ground]
        by_row.append({"row": row, "finite_downward_points": len(selected), "ground_points": len(ground),
                       "ground_range_residual_m": distribution(residuals),
                       "ground_plane_implied_incidence_difference_deg": distribution([
                           r["plane_implied_incidence_angle_difference_deg"] for r in ground
                           if r["plane_implied_incidence_angle_difference_deg"] is not None])})
    return {"snapshot": str(path.resolve()), "snapshot_sha256": hashlib.sha256(data).hexdigest(),
            "cloud": str(cloud_path.resolve()), "cloud_sha256": cloud_hash,
            "stamp_ns": snapshot["stamp_ns"], "scan_sequence": snapshot["scan_sequence"],
            "source_frame": snapshot["source_frame"], "raw_transform": transform,
            "width": points.shape[1], "height": points.shape[0], "row_summaries": by_row,
            "records": records}


def compare_scans(first, second):
    a = {(r["row"], r["column"]): r for r in first["records"]}
    b = {(r["row"], r["column"]): r for r in second["records"]}
    common = [k for k in sorted(set(a) & set(b)) if a[k]["planar_ground_hit"] and b[k]["planar_ground_hit"]]
    x = np.asarray([a[k]["range_residual_m"] for k in common])
    y = np.asarray([b[k]["range_residual_m"] for k in common])
    return {"first_scan": first["scan_sequence"], "second_scan": second["scan_sequence"],
            "raw_transforms_identical": first["raw_transform"] == second["raw_transform"],
            "common_ground_points": len(common),
            "residual_pattern_correlation": float(np.corrcoef(x,y)[0,1]) if len(common)>1 else None,
            "second_minus_first_range_residual_m": distribution(y-x),
            "expected_difference_stddev_for_independent_8mm_noise_m": math.sqrt(2)*.008}


def plot(scans, rows, path):
    # An antialiased diagnostic raster: fixed symmetric scale and explicit
    # clipping keep the same spatial residuals comparable between scans.
    from PIL import Image, ImageDraw, ImageFont
    scale = 2
    canvas = Image.new("RGB", (1500*scale, 1000*scale), "white")
    draw = ImageDraw.Draw(canvas)
    font_path = "/System/Library/Fonts/Supplemental/Arial.ttf"
    def font(size):
        return ImageFont.truetype(font_path, size*scale)
    def text(x,y,s,size=17,fill="black"):
        draw.text((x*scale,y*scale),s,font=font(size),fill=fill)
    def line(points,fill="#777777",width=1):
        draw.line([(int(x*scale),int(y*scale)) for x,y in points],fill=fill,width=width*scale)
    text(40,20,"VLP-16 captured range minus static-mesh range",26)
    text(40,58,"Actual cloud bytes and acquisition-time transform; 8 mm Gaussian noise unchanged.",17)
    text(40,86,"Heatmaps: blue = short return, red = long return; range clipped at +/-1 m, gray = no mesh hit.",17)
    left, width, cell_height = 85, 1320, 20
    def color(value):
        if value is None:
            return (170,170,170)
        t=min(1.0,abs(value))
        end=np.array([195,45,40]) if value>=0 else np.array([35,90,190])
        return tuple(np.round(np.array([250,250,250])*(1-t)+end*t).astype(int))
    for si, scan in enumerate(scans[:2]):
        top=145+si*245
        text(left,top-28,f'Scan {scan["scan_sequence"]}, t = {scan["stamp_ns"]*1e-9:.1f} s',19)
        records={(r["row"],r["column"]):r for r in scan["records"]}
        for ri,row in enumerate(rows):
            text(30,top+ri*cell_height+1,str(row),14)
            for column in range(scan["width"]):
                record=records.get((row,column))
                fill=color(record["range_residual_m"] if record else None)
                x0=left+column*width/scan["width"]
                x1=left+(column+1)*width/scan["width"]
                draw.rectangle((int(x0*scale),int((top+ri*cell_height)*scale),
                                int(x1*scale),int((top+(ri+1)*cell_height)*scale)),fill=fill)
        for column in (0,55,110,165,220,275,330,385,439):
            x=left+column*width/scan["width"]
            text(x-9,top+len(rows)*cell_height+5,str(column),13)
        text(1410,top+len(rows)*cell_height+5,"col",13)
    x0,y0,w,h=100,680,1290,230
    for value in (-1,-.5,0,.5,1):
        y=y0+h*(1-value)/2
        line([(x0,y),(x0+w,y)],"#dddddd")
        text(40,y-10,f'{value:+.1f}',14)
    text(100,640,"Row 4, columns 400-439: two scans at the same stationary pose",20)
    for col in range(400,440,5):
        x=x0+(col-400)*w/39
        line([(x,y0),(x,y0+h)],"#eeeeee")
        text(x-12,y0+h+8,str(col),14)
    for si,scan in enumerate(scans[:2]):
        color_value=("#1663b7","#df671e")[si]
        rs=[r for r in scan["records"] if r["row"]==4 and 400<=r["column"]<=439 and r["range_residual_m"] is not None]
        pts=[(x0+(r["column"]-400)*w/39,y0+h*(1-np.clip(r["range_residual_m"],-1,1))/2) for r in rs]
        line(pts,color_value,2)
        for x,y in pts:
            draw.ellipse(((x-2)*scale,(y-2)*scale,(x+2)*scale,(y+2)*scale),fill=color_value)
        text(1050+si*180,608,f'Scan {scan["scan_sequence"]}',17,color_value)
    text(100,960,"The same direction-dependent pattern persists; this plot alone does not identify the rendering mechanism.",17)
    canvas.resize((1500,1000),Image.Resampling.LANCZOS).save(path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshots",nargs="+",type=Path)
    parser.add_argument("--mesh",type=Path,default=DEFAULT_MESH)
    parser.add_argument("--rows",default="0:8",help="Half-open row interval, e.g. 0:8")
    parser.add_argument("--column-stride",type=int,default=1)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--plot",type=Path)
    args=parser.parse_args()
    lo,hi=map(int,args.rows.split(":")); rows=list(range(lo,hi))
    if not rows or args.column_stride<1:
        parser.error("Invalid row interval or column stride")
    if args.output.exists() or args.plot and args.plot.exists():
        parser.error("Refusing to overwrite outputs")
    vertices,faces=read_mesh(args.mesh); mesh=TriangleRays(vertices,faces)
    scans=[]
    for path in args.snapshots:
        scans.append(analyze(path,mesh,rows,args.column_stride))
        print(json.dumps({"scan":scans[-1]["scan_sequence"],"finite_downward_records":len(scans[-1]["records"])}),flush=True)
    result={"status":"completed","scope":"Organized downward cloud rays versus original static world mesh",
            "mesh":str(args.mesh.resolve()),"mesh_sha256":hashlib.sha256(args.mesh.read_bytes()).hexdigest(),
            "rows":rows,"column_stride":args.column_stride,
            "transform_arithmetic":"Float64 rotation of captured float32 cloud coordinates using the normalized raw double transform; not bit-exact reproduction of the float integrator",
            "noise_stddev_m":.008,"scans":scans,
            "between_scans":[compare_scans(scans[0],scan) for scan in scans[1:]],
            "limitations":["Static collision mesh excludes robot geometry and may differ from rendered visuals.",
                           "Plane-implied angle describes the angle needed for the observed range to hit that plane; it is not a measured rendering ray.",
                           "Correlated angular structure does not alone establish its implementation-level cause."]}
    serialized=json.dumps(result,indent=2,allow_nan=False)+"\n"
    with args.output.open("x") as stream:
        stream.write(serialized)
    if args.plot:
        plot(scans,rows,args.plot)
    print(json.dumps({"status":"completed","output":str(args.output),"between_scans":result["between_scans"]}))


if __name__=="__main__":
    main()
