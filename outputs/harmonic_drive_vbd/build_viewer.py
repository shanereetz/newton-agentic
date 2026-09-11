"""Build a self-contained offline replay of the actual Newton trajectory."""
import argparse,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--data',type=Path,default=ROOT/'results_rom/trajectory.json');p.add_argument('--out',type=Path,default=ROOT/'replay_rom.html');p.add_argument('--inline',type=Path);args=p.parse_args()
data=json.loads(args.data.read_text());template=(ROOT/'viewer_template.html').read_text()
from html import escape
template=template.replace('Newton · VBD',escape('Newton · '+data['solver']))
fragment=template.replace('__DATA__',json.dumps(data,separators=(',',':')))
style='''<style>:root{color-scheme:light dark;--viz-series-1:light-dark(#44799b,#76aacd);--viz-series-2:light-dark(#a95123,#eca36d);--viz-series-3:light-dark(#33866c,#78c5a4);--viz-series-4:light-dark(#a02671,#fa99d0)}body{font:15px system-ui;margin:20px auto;padding:0 18px;max-width:960px;color:light-dark(#253242,#e5ebf3);background:light-dark(#f7f9fc,#151b24)}.viz-row,.viz-controls{display:flex;gap:14px;align-items:center;flex-wrap:wrap;margin:12px 0}.text-small{font-size:13px}.btn,select{font:inherit;padding:8px 14px;border:1px solid #8888;border-radius:7px;background:transparent;color:inherit;cursor:pointer}.form-label{display:flex;align-items:center;gap:8px}.form-range{width:170px}.form-check{display:flex;align-items:center;gap:7px}.tabular-nums{font-variant-numeric:tabular-nums}canvas{touch-action:pan-y}@media(max-width:420px){.form-range{width:110px}}</style>'''
args.out.write_text('<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Harmonic drive — Newton simulation replay</title>'+style+'<body>'+fragment+'</body></html>')
if args.inline:
    # Retain real frames only; no kinematic synthesis or invented force samples.
    data['frames']=data['frames'][::30]+([data['frames'][-1]] if (len(data['frames'])-1)%30 else [])
    data.pop('config',None)
    text=template.replace('__DATA__',json.dumps(data,separators=(',',':')))
    assert len(text.encode())<1_000_000,len(text)
    args.inline.write_text(text)
print('Built offline replay.')
