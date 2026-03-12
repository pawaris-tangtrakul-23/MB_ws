#!/usr/bin/env python3
"""Calculate MSE and RMSE for each axis and total from flight CSV files."""

import sys
import os
import glob
import csv
import numpy as np


def load_csv(csv_path):
    """Load CSV into a dict of numpy arrays."""
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        headers = next(reader)
        rows = [row for row in reader]
    data = {}
    for i, h in enumerate(headers):
        data[h] = np.array([float(row[i]) for row in rows])
    return data


def calc_errors(csv_path):
    """Calculate MSE and RMSE per axis and total for one CSV file."""
    d = load_csv(csv_path)
    fname = os.path.basename(csv_path)

    err_x = d['curr_x'] - d['tgt_x']
    err_y = d['curr_y'] - d['tgt_y']
    err_z = d['curr_z'] - d['tgt_z']
    err_total = np.sqrt(err_x**2 + err_y**2 + err_z**2)

    mse_x = np.mean(err_x**2)
    mse_y = np.mean(err_y**2)
    mse_z = np.mean(err_z**2)
    mse_total = np.mean(err_total**2)

    rmse_x = np.sqrt(mse_x)
    rmse_y = np.sqrt(mse_y)
    rmse_z = np.sqrt(mse_z)
    rmse_total = np.sqrt(mse_total)

    return {
        'file': fname,
        'samples': len(d['time_s']),
        'duration': d['time_s'][-1],
        'mse_x': mse_x, 'mse_y': mse_y, 'mse_z': mse_z, 'mse_total': mse_total,
        'rmse_x': rmse_x, 'rmse_y': rmse_y, 'rmse_z': rmse_z, 'rmse_total': rmse_total,
    }


def print_report(results):
    """Print a formatted table of all results."""
    # Header
    print(f"{'File':<60} {'Samples':>7} {'Dur(s)':>7}  "
          f"{'MSE_x':>10} {'MSE_y':>10} {'MSE_z':>10} {'MSE_tot':>10}  "
          f"{'RMSE_x':>10} {'RMSE_y':>10} {'RMSE_z':>10} {'RMSE_tot':>10}")
    print("-" * 170)

    for r in results:
        print(f"{r['file']:<60} {r['samples']:>7} {r['duration']:>7.2f}  "
              f"{r['mse_x']:>10.6f} {r['mse_y']:>10.6f} {r['mse_z']:>10.6f} {r['mse_total']:>10.6f}  "
              f"{r['rmse_x']:>10.6f} {r['rmse_y']:>10.6f} {r['rmse_z']:>10.6f} {r['rmse_total']:>10.6f}")

    # Summary across all files
    if len(results) > 1:
        print("-" * 170)
        print(f"{'AVERAGE':<60} {'':>7} {'':>7}  "
              f"{np.mean([r['mse_x'] for r in results]):>10.6f} "
              f"{np.mean([r['mse_y'] for r in results]):>10.6f} "
              f"{np.mean([r['mse_z'] for r in results]):>10.6f} "
              f"{np.mean([r['mse_total'] for r in results]):>10.6f}  "
              f"{np.mean([r['rmse_x'] for r in results]):>10.6f} "
              f"{np.mean([r['rmse_y'] for r in results]):>10.6f} "
              f"{np.mean([r['rmse_z'] for r in results]):>10.6f} "
              f"{np.mean([r['rmse_total'] for r in results]):>10.6f}")


def save_csv_report(results, output_path):
    """Save results as a CSV file."""
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'file', 'samples', 'duration_s',
            'MSE_x', 'MSE_y', 'MSE_z', 'MSE_total',
            'RMSE_x', 'RMSE_y', 'RMSE_z', 'RMSE_total',
        ])
        for r in results:
            writer.writerow([
                r['file'], r['samples'], f"{r['duration']:.2f}",
                f"{r['mse_x']:.6f}", f"{r['mse_y']:.6f}", f"{r['mse_z']:.6f}", f"{r['mse_total']:.6f}",
                f"{r['rmse_x']:.6f}", f"{r['rmse_y']:.6f}", f"{r['rmse_z']:.6f}", f"{r['rmse_total']:.6f}",
            ])
    print(f"\nSaved CSV report: {output_path}")


def main():
    docs_dir = os.path.dirname(os.path.abspath(__file__))

    if len(sys.argv) > 1:
        csv_files = sys.argv[1:]
    else:
        csv_files = sorted(glob.glob(os.path.join(docs_dir, 'flight_*.csv')))
        if not csv_files:
            print("No flight CSV files found in docs/")
            print(f"Usage: python3 {sys.argv[0]} [path_to_csv ...]")
            return
        print(f"Found {len(csv_files)} flight CSV files\n")

    results = []
    for csv_path in csv_files:
        if not os.path.isfile(csv_path):
            print(f"File not found: {csv_path}")
            continue
        results.append(calc_errors(csv_path))

    if not results:
        return

    print_report(results)

    # Save report CSV
    report_path = os.path.join(docs_dir, 'error_report.csv')
    save_csv_report(results, report_path)


if __name__ == '__main__':
    main()
