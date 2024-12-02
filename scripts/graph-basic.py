import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import re

def parse_data(line):
    """
    Parses a line to extract the integer after 'd:' if present.
    Returns None if no valid data is found.
    """
    match = re.match(r'^d:\s*(\d+)', line)
    return int(match.group(1)) if match else None

def live_plot():
    """
    Real-time plotting from stdin.
    """
    global start_time, y_data, x_data

    def update(frame):
        # Read input from stdin
        try:
            line = sys.stdin.readline()
            if not line:
                return

            # Parse the data
            value = parse_data(line.strip())
            if value is not None:
                elapsed_time = time.time() - start_time
                x_data.append(elapsed_time)
                y_data.append(value)

                # Update plot
                ax.clear()
                ax.plot(x_data, y_data, marker='o', linestyle='-')
                ax.set_title("Live Plot of d: Values")
                ax.set_xlabel("Time (s)")
                ax.set_ylabel("Values")
                ax.grid(True)

        except KeyboardInterrupt:
            plt.close()

    # Initialize data
    x_data, y_data = [], []
    start_time = time.time()

    # Set up plot
    fig, ax = plt.subplots()
    ani = animation.FuncAnimation(fig, update, interval=100)

    # Display the plot
    plt.show()

def file_plot(filename):
    """
    Instant plotting from a file.
    """
    x_data, y_data = [], []

    try:
        with open(filename, 'r') as file:
            for line in file:
                value = parse_data(line.strip())
                if value is not None:
                    x_data.append(len(x_data))  # Use index as x-axis
                    y_data.append(value)

        # Plot the data
        plt.plot(x_data, y_data, marker='o', linestyle='-')
        plt.title("Instant Plot of d: Values")
        plt.xlabel("Index")
        plt.ylabel("Values")
        plt.grid(True)
        plt.show()

    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        # If a filename is provided, read and plot the file
        file_plot(sys.argv[1])
    else:
        # Otherwise, use live plotting mode
        live_plot()
