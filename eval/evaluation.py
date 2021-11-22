import numpy as np

def load_errors():
    errors = []
    with open('range_errors.txt') as f:
        file = f.readlines()
    for line in file:
        if line.strip():
            errors.append(float(line.strip()))
    return np.array(errors)

def calculate_avg_error(errors):
    return np.sum(errors)/errors.size

def calculate_avg_pos_errors(errors):
    return np.sum(errors[errors > 0]/errors[errors > 0].size)

def calculate_avg_neg_errors(errors):
    return np.sum(errors[errors < 0]/errors[errors < 0].size)

def calculate_max_neg_error(errors):
    return np.min(errors[errors < 0])

def calculate_min_neg_error(errors):
    return np.max(errors[errors < 0])

def save_error_plot(errors):
    from matplotlib import pyplot as plt
    fig, ax = plt.subplots(nrows=1, ncols=1)
    ax.set_xlabel('Time (t_s)')
    ax.set_ylabel('Range Error (m)')
    ax.set_title('Range Errors (m/t_s)')

    ax.plot(np.arange(errors.size), errors)
    fig.savefig('range_errors.png')
    plt.close(fig)

def plot_vel_rel():
    y = [0.1368, 0.0959, 0.0770, 0.0747]
    x = [0.26, 0.195, 0.13, 0.065]

    from matplotlib import pyplot as plt
    fig, ax = plt.subplots(nrows=1, ncols=1)
    ax.set_xlabel('Linear velocity (v)')
    ax.set_ylabel('Absolute Average Range Error (m/t_s)')
    ax.set_title('Linear velocity relationship with the absolute average range error')

    ax.plot(x, y)
    fig.savefig('vel_rel.png')
    plt.close(fig)

def main():
    errors = load_errors()

    avg_range_error = calculate_avg_error(errors)
    avg_pos_error = calculate_avg_pos_errors(errors)
    avg_neg_error = calculate_avg_neg_errors(errors)
    max_neg_error = calculate_max_neg_error(errors)
    min_neg_error = calculate_min_neg_error(errors)

    # Account for precision to the threshold distance
    avg_range_error = avg_range_error + 0.05
    avg_pos_error = max(avg_pos_error - 0.05, avg_pos_error)
    avg_neg_error = min(avg_neg_error + 0.05, avg_neg_error)
    max_neg_error = max_neg_error + 0.05
    min_neg_error = min_neg_error - 0.05

    save_error_plot(errors)
    print(" ---- Evaluation Results ----")
    print("Avg Range Error = ", avg_range_error, " (+/- 0.05)")
    print("Avg Positive Range Error = ", avg_pos_error, " (+/- 0.05)")
    print("Avg Negative Range Error = ", avg_neg_error, " (+/- 0.05)")
    print("Maximum Negative Error = ", max_neg_error, " (+/- 0.05)")
    print("Minimum Negative Error = ", min_neg_error, " (+/- 0.05)")
    print(" ---- Evaluation done! ----")
    print("Don't forget to check range_errors.png plot")

if __name__ == "__main__":
    main()
