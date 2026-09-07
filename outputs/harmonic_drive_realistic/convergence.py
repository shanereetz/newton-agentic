"""Run independent CUDA refinement cases; report differences, not certification."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
CASES = {
    'reference': [],
    'half_dt': ['--dt', '0.00025'],
    'double_iterations': ['--iterations', '200'],
    'half_voxel': ['--voxel', '0.0000125'],
    'refined_mesh': ['--samples', '16', '--layers', '3', '--axial-refine', '2'],
    'no_ring_control': ['--no-ring-contact'],
}

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cases', nargs='+', choices=CASES, default=list(CASES))
    p.add_argument('--duration', type=float, default=8.)
    p.add_argument('--out', type=Path, default=ROOT / 'runs' / 'convergence')
    p.add_argument('--dry-run', action='store_true')
    args = p.parse_args()
    results = {}
    for name in args.cases:
        out = args.out / name
        command = [sys.executable, str(ROOT / 'realistic.py'), '--duration',
                   str(args.duration), '--out', str(out), *CASES[name]]
        print(subprocess.list2cmdline(command), flush=True)
        if args.dry_run:
            continue
        out.mkdir(parents=True, exist_ok=True)
        with (out / 'console.log').open('w') as log:
            completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
        if completed.returncode:
            raise RuntimeError(f'{name} failed; inspect {out / "console.log"}')
        with (out / 'metrics.csv').open() as f:
            rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]
        start = min(rows, key=lambda row: abs(row['time_s'] - 5.))
        final = rows[-1]
        dx = final['input_rad'] - start['input_rad']
        results[name] = {
            'output_per_input_after_assembly': (final['output_rad']-start['output_rad'])/dx,
            'min_J': min(r['min_J'] for r in rows),
            'peak_penetration_um': max(r['max_contact_penetration_um'] for r in rows),
            'peak_von_mises_MPa': max(r['max_von_mises_MPa'] for r in rows),
            'final_output_rad': final['output_rad'],
            'final_elastic_energy_J': final['elastic_energy_J'],
        }
        (args.out / 'comparison.json').write_text(json.dumps(results, indent=2))
    if results:
        print(json.dumps(results, indent=2))

if __name__ == '__main__':
    main()
