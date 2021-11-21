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

def calculate_absolute_error(errors):
    return np.sum(errors)

def save_error_plot(errors):
    from matplotlib import pyplot as plt
    fig, ax = plt.subplots(nrows=1, ncols=1)
    ax.set_xlabel('Time (t_s)')
    ax.set_ylabel('Range Error')
    ax.set_title('Range Errors')

    ax.plot(np.arange(errors.size), errors)
    fig.savefig('range_errors.png')
    plt.close(fig)

def main():
    errors = load_errors()

    avg_range_error = calculate_avg_error(errors)
    avg_pos_error = calculate_avg_pos_errors(errors)
    avg_neg_error = calculate_avg_neg_errors(errors)
    absolute_error = calculate_absolute_error(errors)

    save_error_plot(errors)
    print(" ---- Evaluation Results ----")
    print("Avg Range Error = ", avg_range_error)
    print("Avg Positive Range Error = ", avg_pos_error)
    print("Avg Negative Range Error = ", avg_neg_error)
    print("Accumulated Error = ", absolute_error)
    print(" ---- Evaluation done! ----")
    print("Don't forget to check range_errors.png plot")

if __name__ == "__main__":
    main()
