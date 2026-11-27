#!/usr/bin/env python3
"""Compare native static 128/512 lidar captures on the same ground rays."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from analyze_lidar_mesh_grid import analyze
from analyze_scan_endpoints import DEFAULT_MESH, TriangleRays, read_mesh


def metrics(values):
    values = np.asarray(values, dtype=np.float64)
    absolute = np.abs(values)
    return {"count": len(values), "signed_mean_m": float(np.mean(values)),
            "rms_m": float(np.sqrt(np.mean(values**2))),
            "absolute_p95_m": float(np.quantile(absolute,.95)), "absolute_max_m": float(np.max(absolute)),
            "signed_min_m": float(np.min(values)), "signed_max_m": float(np.max(values))}


def vertical_error(record):
    # n dot(endpoint-hit) / n.z equals endpoint.z minus the height of the
    # intersected triangle plane at endpoint.x/y, including the slight slope.
    return record["endpoint_height_above_intersected_plane_m"] / record["mesh_hit"]["normal_world"][2]


def summarize(records):
    return {"range_error": metrics([r["range_residual_m"] for r in records]),
            "vertical_plane_height_error": metrics([vertical_error(r) for r in records]),
            "mesh_hit_height_m": metrics([r["mesh_hit"]["point_world"][2] for r in records]),
            "endpoint_z_at_least_0_20m": sum(r["endpoint_world"][2] >= .2 for r in records),
            "endpoint_z_at_least_0_25m": sum(r["endpoint_world"][2] >= .25 for r in records)}


def make_result(scans):
    maps=[{(r["row"],r["column"]):r for r in scan["records"]} for scan in scans]
    common=[key for key in sorted(set(maps[0]) & set(maps[1]))
            if all(m[key]["planar_ground_hit"] for m in maps)]
    if not common:
        raise ValueError("No common near-horizontal ground rays")
    if scans[0]["raw_transform"] != scans[1]["raw_transform"]:
        raise ValueError("Probe poses differ")
    result={"status":"completed","scope":"static_native_sensor_probe_precision_comparison",
            "baseline":"internal128","updated":"internal512","output_dimensions":[440,16],
            "noise_stddev_m":.008,"same_world_pose":True,
            "ground_selection":"Common finite downward cloud records whose first static-mesh hit has abs(normal.z)>0.99 in both captures; rows0..7, all440columns",
            "common_ground_rays":len(common),"selected_cloud_points_per_capture":[len(s["records"]) for s in scans],
            "vertical_error_definition":"endpoint.z minus the intersected triangle plane height at the endpoint XY; n dot(endpoint-hit)/n.z",
            "metrics":{},"rows0_to6_metrics":{},"focus_points":[],"row4_curve":[],
            "inputs":[{k:scan[k] for k in ("snapshot","snapshot_sha256","cloud","cloud_sha256","raw_transform")}
                      for scan in scans],
            "limitations":["One retained capture per renderer, one fixed pose and one static world; no statistical repeat-run guarantee.",
                           "Near-horizontal rays amplify range error; the full sample includes six row7 ground returns.",
                           "The sensor-only probe has no robot body, mapper, planner or mission execution.",
                           "Fewer high endpoints show reduced direct false-high returns; TSDF ray traversal, truncation and accumulated occupancy are not evaluated here.",
                           "Internal renderer size must be established from the corresponding process configuration/logs; the cloud still has440x16points."]}
    for tag,m in zip(("internal128","internal512"),maps):
        result["metrics"][tag]=summarize([m[k] for k in common])
        result["rows0_to6_metrics"][tag]=summarize([m[k] for k in common if k[0]<=6])
    for point_index in (2185,2186):
        key=divmod(point_index,440)
        item={"point_index":point_index,"row":key[0],"column":key[1]}
        for tag,m in zip(("internal128","internal512"),maps):
            r=m[key]
            item[tag]={"measured_range_m":r["measured_range_m"],"mesh_range_m":r["mesh_hit"]["range_m"],
                       "range_error_m":r["range_residual_m"],"vertical_error_m":vertical_error(r),
                       "endpoint_world":r["endpoint_world"],"mesh_hit_world":r["mesh_hit"]["point_world"]}
        result["focus_points"].append(item)
    for col in range(413,440):
        item={"column":col}
        for tag,m in zip(("internal128","internal512"),maps):
            r=m[(4,col)]
            item[tag]={"vertical_error_m":vertical_error(r),"range_error_m":r["range_residual_m"],
                       "endpoint_z_m":r["endpoint_world"][2],"mesh_hit_z_m":r["mesh_hit"]["point_world"][2]}
        result["row4_curve"].append(item)
    result["improvement"]={}
    for category in ("range_error","vertical_plane_height_error"):
        result["improvement"][category]={}
        for metric in ("rms_m","absolute_p95_m","absolute_max_m"):
            before=result["metrics"]["internal128"][category][metric]
            after=result["metrics"]["internal512"][category][metric]
            result["improvement"][category][metric]={"reduction_fraction":1-after/before,"baseline_over_updated":before/after}
    return result


def plot(result,path):
    from PIL import Image,ImageDraw,ImageFont
    scale=2
    canvas=Image.new("RGB",(1500*scale,1050*scale),"white");d=ImageDraw.Draw(canvas)
    fontfile="/System/Library/Fonts/Supplemental/Arial.ttf"
    colors={"internal128":"#2167ad","internal512":"#db6a23"}
    def text(x,y,t,size=18,fill="#222222"):
        d.text((x*scale,y*scale),t,font=ImageFont.truetype(fontfile,size*scale),fill=fill)
    def line(points,color="#dddddd",width=1):
        d.line([(int(x*scale),int(y*scale)) for x,y in points],fill=color,width=width*scale)
    text(50,25,"Native static lidar: internal 128 vs 512 pixels",29)
    text(50,73,f'Common near-horizontal ground rays: {result["common_ground_rays"]} / 3520 downward points per capture',20)
    text(50,105,"Same world pose, original mine, 440 x 16 output and 0.008 m Gaussian noise retained.",18)
    text(50,151,"Absolute error metrics (mm)",22)
    text(650,154,"RMS",20);text(880,154,"95th percentile",20);text(1160,154,"Maximum",20)
    yy=197
    for category,label in (("range_error","Range"),("vertical_plane_height_error","Vertical plane height")):
        for tag in ("internal128","internal512"):
            text(55,yy,f'{label} / {tag.replace("internal", "")}',19,colors[tag])
            for x,metric in zip((650,880,1160),("rms_m","absolute_p95_m","absolute_max_m")):
                text(x,yy,f'{result["metrics"][tag][category][metric]*1000:.1f}',20,colors[tag])
            yy+=36
        yy+=15
    text(50,383,"Range maxima include near-horizon returns. Height measures deviation from the intersected ground plane.",16)
    text(50,426,"Row 4, columns 413-439: signed vertical error",22)
    text(1110,426,"128",19,colors["internal128"]);text(1220,426,"512",19,colors["internal512"])
    x0,y0,w,h=105,475,1280,275
    for value in (-80,-40,0,40,80):
        y=y0+h*(80-value)/160
        line([(x0,y),(x0+w,y)],"#bbbbbb" if value==0 else "#e4e4e4")
        text(32,y-11,f'{value:+d}',16)
    text(30,446,"mm",16)
    for col in (413,417,421,425,426,430,434,439):
        x=x0+(col-413)*w/26
        line([(x,y0),(x,y0+h)],"#eeeeee");text(x-13,y0+h+10,str(col),15)
    for tag,color in colors.items():
        points=[(x0+(r["column"]-413)*w/26,y0+h*(80-r[tag]["vertical_error_m"]*1000)/160)
                for r in result["row4_curve"]]
        line(points,color,3)
        for x,y in points:
            d.ellipse(((x-3)*scale,(y-3)*scale,(x+3)*scale,(y+3)*scale),fill=color)
    text(50,812,"Focus rays",21)
    for index,item in enumerate(result["focus_points"]):
        a,b=item["internal128"],item["internal512"]
        text(55,850+index*40,f'Point {item["point_index"]} (row {item["row"]}, col {item["column"]}): '
             f'height error {a["vertical_error_m"]*1000:+.2f} -> {b["vertical_error_m"]*1000:+.2f} mm; '
             f'endpoint z {a["endpoint_world"][2]:.4f} -> {b["endpoint_world"][2]:.4f} m',18)
    old=result["metrics"]["internal128"];new=result["metrics"]["internal512"]
    text(50,950,f'Endpoints at z >= 0.20 m: {old["endpoint_z_at_least_0_20m"]} -> {new["endpoint_z_at_least_0_20m"]}; '
         f'at z >= 0.25 m: {old["endpoint_z_at_least_0_25m"]} -> {new["endpoint_z_at_least_0_25m"]} '
         f'(same {result["common_ground_rays"]} ground rays).',19)
    text(50,993,"This proves better static-scan geometry at this pose, not TSDF-map correctness or a completed autonomous mission.",17)
    canvas.resize((1500,1050),Image.Resampling.LANCZOS).save(path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline",type=Path);parser.add_argument("updated",type=Path)
    parser.add_argument("--output",required=True,type=Path);parser.add_argument("--plot",type=Path)
    parser.add_argument("--mesh",type=Path,default=DEFAULT_MESH)
    parser.add_argument("--precomputed-cache",type=Path,help="Optional previous analyze() results; hashes are checked against both inputs")
    args=parser.parse_args()
    if args.output.exists() or args.plot and args.plot.exists():
        parser.error("Refusing to overwrite outputs")
    paths=(args.baseline,args.updated)
    if args.precomputed_cache:
        scans=json.loads(args.precomputed_cache.read_text())
        if len(scans)!=2:
            raise ValueError("Expected exactly two cached scans")
        for scan,path in zip(scans,paths):
            if scan["snapshot_sha256"]!=hashlib.sha256(path.read_bytes()).hexdigest():
                raise ValueError("Cached snapshot hash mismatch")
            cloud=path.parent/json.loads(path.read_text())["cloud"]["data_file"]
            if scan["cloud_sha256"]!=hashlib.sha256(cloud.read_bytes()).hexdigest():
                raise ValueError("Cached cloud hash mismatch")
    else:
        vertices,faces=read_mesh(args.mesh);mesh=TriangleRays(vertices,faces)
        scans=[analyze(path,mesh,list(range(8)),1) for path in paths]
    result=make_result(scans)
    result["mesh"]={"path":str(args.mesh.resolve()),"sha256":hashlib.sha256(args.mesh.read_bytes()).hexdigest()}
    data=json.dumps(result,indent=2,allow_nan=False)+"\n"
    with args.output.open("x") as stream:stream.write(data)
    if args.plot:plot(result,args.plot)
    print(json.dumps({"status":"completed","output":str(args.output),"sample_count":result["common_ground_rays"],
                      "metrics":result["metrics"]}))


if __name__=="__main__":main()
