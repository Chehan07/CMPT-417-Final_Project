# Libraries
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


# Files
RESULTS_DIRECTORY_NAME = 'results\\'
CBS_RESULT_FILE_NAME = 'cbs_result.csv'
CBS_DISJOINT_RESULT_FILE_NAME = 'cbs_disjoint_result.csv'
CBS_SIPP_RESULT_FILE_NAME = 'cbs_sipp_result.csv'
CBS_SIPP_DISJOINT_RESULT_FILE_NAME = 'cbs_sipp_disjoint_result.csv'

GENERATED_NODES_FIGURE_NAME = 'generated_nodes.png'
EXPANDED_NODES_FIGURE_NAME = 'expanded_nodes.png'
COST_FIGURE_NAME = 'cost.png'
CPU_TIME_FIGURE_NAME = 'cpu_time.png'
GENERATED_NODES_REMOVE_EXTREME_FIGURE_NAME = 'generated_nodes_remove_extreme.png'
EXPANDED_NODES_REMOVE_EXTREME_FIGURE_NAME = 'expanded_nodes_remove_extreme.png'
CPU_TIME_REMOVE_EXTREME_FIGURE_NAME = 'cpu_time_remove_extreme.png'

# Check files
#
# Returns
# True: all the csv files exist
# False: one of the csv files does not exist
def check_files():
    if not os.path.exists(RESULTS_DIRECTORY_NAME + CBS_RESULT_FILE_NAME):
        return False
    if not os.path.exists(RESULTS_DIRECTORY_NAME + CBS_DISJOINT_RESULT_FILE_NAME):
        return False
    if not os.path.exists(RESULTS_DIRECTORY_NAME + CBS_SIPP_RESULT_FILE_NAME):
        return False
    if not os.path.exists(RESULTS_DIRECTORY_NAME + CBS_SIPP_DISJOINT_RESULT_FILE_NAME):
        return False
    
    return True


# Generate the figure
def generate_figure(datasets, title, ylabel, legend_strs, column_name, instance_names, file_name):
    plt.figure(figsize=(10, 5))
    plt.title(title)
    plt.xlabel('Instance')
    plt.ylabel(ylabel)

    plt.plot(instance_names, datasets[0][column_name], 'r-')
    plt.plot(instance_names, datasets[1][column_name], 'g-')
    plt.plot(instance_names, datasets[2][column_name], 'b-')
    plt.plot(instance_names, datasets[3][column_name], 'k-')

    plt.legend(legend_strs)

    plt.savefig(RESULTS_DIRECTORY_NAME + file_name)


# Main function
if __name__ == "__main__":
    # Check files
    if not check_files():
        print('Unable to generate graphs: one of the csv files does not exist')
    else:
        # Load the csv files
        header_name_strs = ['File Names', 'Generated Nodes', 'Expanded Nodes', 'Costs', 'CPU Times']
        cbs_dataset = pd.read_csv(RESULTS_DIRECTORY_NAME + CBS_RESULT_FILE_NAME, names=header_name_strs)
        cbs_disjoint_dataset = pd.read_csv(RESULTS_DIRECTORY_NAME + CBS_DISJOINT_RESULT_FILE_NAME, names=header_name_strs)
        cbs_sipp_dataset = pd.read_csv(RESULTS_DIRECTORY_NAME + CBS_SIPP_RESULT_FILE_NAME, names=header_name_strs)
        cbs_sipp_disjoint_dataset = pd.read_csv(RESULTS_DIRECTORY_NAME + CBS_SIPP_DISJOINT_RESULT_FILE_NAME, names=header_name_strs)
        datasets = [cbs_dataset, cbs_disjoint_dataset, cbs_sipp_dataset, cbs_sipp_disjoint_dataset]

        # Set the figure info
        instance_names = np.linspace(1, 50)
        legend_strs = ['CBS with Space-time A*', 'CBS with Space-time A* and Disjoint Splitting', 'CBS with SIPP', 'CBS with SIPP and Disjoint Splitting']

        # Generate the generated nodes figure
        title = header_name_strs[1]
        column_name = title
        ylabel = 'Number of ' + column_name
        file_name = GENERATED_NODES_FIGURE_NAME
        generate_figure(datasets, title, ylabel, legend_strs, column_name, instance_names, file_name)

        # Generate the expanded nodes figure
        title = header_name_strs[2]
        column_name = title
        ylabel = 'Number of ' + column_name
        file_name = EXPANDED_NODES_FIGURE_NAME
        generate_figure(datasets, title, ylabel, legend_strs, column_name, instance_names, file_name)

        # Generate the costs figure
        title = header_name_strs[3]
        column_name = title
        ylabel = 'Cost'
        file_name = COST_FIGURE_NAME
        generate_figure(datasets, title, ylabel, legend_strs, column_name, instance_names, file_name)

        # Generate the cpu times figure
        title = header_name_strs[4]
        column_name = title
        ylabel = 'CPU Time (s)'
        file_name = CPU_TIME_FIGURE_NAME
        generate_figure(datasets, title, ylabel, legend_strs, column_name, instance_names, file_name)

        # Remove the extreme case
        max_index = 46
        cbs_dataset.drop(max_index, inplace=True)
        cbs_disjoint_dataset.drop(max_index, inplace=True)
        cbs_sipp_dataset.drop(max_index, inplace=True)
        cbs_sipp_disjoint_dataset.drop(max_index, inplace=True)
        instance_names = np.delete(instance_names, max_index)

        # Generate the generated nodes figure
        title = header_name_strs[1] + ' (Remove Extreme Case)'
        column_name = header_name_strs[1]
        ylabel = 'Number of ' + column_name
        file_name = GENERATED_NODES_REMOVE_EXTREME_FIGURE_NAME
        generate_figure(datasets, title, ylabel, legend_strs, column_name, instance_names, file_name)

        # Generate the expanded nodes figure
        title = header_name_strs[2] + ' (Remove Extreme Case)'
        column_name = header_name_strs[2]
        ylabel = 'Number of ' + column_name
        file_name = EXPANDED_NODES_REMOVE_EXTREME_FIGURE_NAME
        generate_figure(datasets, title, ylabel, legend_strs, column_name, instance_names, file_name)

        # Generate the cpu times figure
        title = header_name_strs[4] + ' (Remove Extreme Case)'
        column_name = header_name_strs[4]
        ylabel = 'CPU Time (s)'
        file_name = CPU_TIME_REMOVE_EXTREME_FIGURE_NAME
        generate_figure(datasets, title, ylabel, legend_strs, column_name, instance_names, file_name)
