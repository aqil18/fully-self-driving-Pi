import pandas as pd, os

ROOT = os.path.dirname(os.path.abspath(__file__))


frames = []
for i in range(1, 9):
    d = f"datasets/{i}"
    d = os.path.join(ROOT, d)
    df = pd.read_csv(f"{d}/labels/labels.csv")
    df["filename"] = df["filename"].apply(lambda f: f"{d}/images/{f}")
    frames.append(df)

merged = pd.concat(frames, ignore_index=True)
out_dir = os.path.join(ROOT, "datasets/merged/labels")
os.makedirs(out_dir, exist_ok=True)
merged.to_csv(os.path.join(out_dir, "labels.csv"), index=False)
print(f"Total: {len(merged)} rows")