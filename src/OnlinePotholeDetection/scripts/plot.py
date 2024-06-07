import pandas as pd
import matplotlib.pyplot as plt
import tikzplotlib


def plot_csv_files(files, labels):
    plt.figure(figsize=(10, 6))
    for file, label in zip(files, labels):
        # Read the CSV files into pandas DataFrames
        df = pd.read_csv(file)

        # Extract the "step" and "value" columns from each DataFrame
        step1, value1 = df["Step"], df["Value"]

        # Plot the data
        plt.plot(step1, value1, label=label)
    plt.xlabel("Step")
    plt.ylabel("Value")
    plt.title("Step vs. Value")
    plt.legend()
    plt.grid(True)

    # Export the figure to a TikZ figure file
    tikzplotlib.save('mioufigures' + ".tex")

    # Save the figure as PNG
    plt.savefig('miou-figures' + ".png", dpi=300)

    # Save the figure as SVG
    plt.savefig('miou-figures' + ".svg")


if __name__ == "__main__":
    files = [
        # "/home/samwise/catkin_ws/src/OnlinePotholeDetection/scripts/DetectionNode/sessions/2023-07-31_11-07-14-train/run-2023-07-31_11-07-14-train-tag-miou.csv",
        # "/home/samwise/catkin_ws/src/OnlinePotholeDetection/scripts/DetectionNode/sessions/2023-07-31_11-23-05-train/run-2023-07-31_11-23-05-train-tag-miou.csv",
        "/home/samwise/catkin_ws/src/OnlinePotholeDetection/scripts/DetectionNode/sessions/2023-07-31_10-30-38-dropout-train/run-2023-07-31_10-30-38-dropout-train-tag-miou.csv",
        "/home/samwise/catkin_ws/src/OnlinePotholeDetection/scripts/DetectionNode/sessions/2023-07-31_10-47-17-dropout-train/run-2023-07-31_10-47-17-dropout-train-tag-miou.csv"
    ]
    labels = [
        # 'U-Net',
        # 'U-Net*',
        'Dropout',
        'Dropout*',
    ]

    plot_csv_files(files, labels)
