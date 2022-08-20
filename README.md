# File Description
* instances: a folder that stores map instances
* results: a folder that stores results
* run_experiments.py: a start of this program
* visualize.py: visualize the paths of agents
* single_agent_planner.py: implement Space-time A*
* single_agent_planner_sipp.py: implement SIPP
* cbs.py: implement CBS
* generate_graphs.py: generate graphs for analysis
<br />
<br />
<br />

# Result Description
* cbs_result.csv: CBS with Space-time A* result
* cbs_disjoint_result.csv: CBS with Space-time A* and disjoint splitting result
* cbs_sipp_disjoint_result.csv: CBS with SIPP result
* cbs_sipp_result.csv: CBS with SIPP and disjoint splitting result
* generated_nodes.png: generated nodes graph
* expanded_nodes.png: expended nodes graph
* cost.png: cost graph
* cpu_time.png: CPU time graph
* generated_nodes_remove_extreme.png: generated nodes graph (remove extreme case)
* expanded_nodes_remove_extreme.png: expended nodes graph (remove extreme case)
* cpu_time_remove_extreme.png: CPU time graph (remove extreme case)
<br />
<br />
<br />

# Execution Guideline
## Run an Instance
### CBS with Space-time A*
```shell
python run_experiments.py --instance instances/test_01.txt --solver CBS
```

### CBS with Space-time A* and Disjoint Splitting
```shell
python run_experiments.py --instance instances/test_01.txt --solver CBS --disjoint
```

### CBS with SIPP
```shell
python run_experiments.py --instance instances/test_01.txt --solver CBS_SIPP
```

### CBS with SIPP and Disjoint Splitting
```shell
python run_experiments.py --instance instances/test_01.txt --solver CBS_SIPP --disjoint
```
<br />
<br />
<br />

## Run All Instances
### CBS with Space-time A*
```shell
python run_experiments.py --instance "instances/test_*" --solver CBS --batch
```

### CBS with Space-time A* and Disjoint Splitting
```shell
python run_experiments.py --instance "instances/test_*" --solver CBS --disjoint --batch
```

### CBS with SIPP
```shell
python run_experiments.py --instance "instances/test_*" --solver CBS_SIPP --batch
```

### CBS with SIPP and Disjoint Splitting
```shell
python run_experiments.py --instance "instances/test_*" --solver CBS_SIPP --disjoint --batch
```
<br />
<br />
<br />

## Generate Graphs for Analysis
```shell
python generate_graphs.py
```