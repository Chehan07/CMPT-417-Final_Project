# Command

## Run an Instance
### Execute CBS with Space-time A*
```shell
python run_experiments.py --instance instances/test_1.txt --solver CBS
```

### Execute CBS with Space-time A* and Disjoint Splitting
```shell
python run_experiments.py --instance instances/test_1.txt --solver CBS --disjoint
```

### Execute CBS with SIPP
```shell
python run_experiments.py --instance instances/test_1.txt --solver CBS_SIPP
```

### Execute CBS with SIPP and Disjoint Splitting
```shell
python run_experiments.py --instance instances/test_1.txt --solver CBS_SIPP --disjoint
```


## Run All Instances
### Execute CBS with Space-time A*
```shell
python run_experiments.py --instance "instances/test_*" --solver CBS --batch
```

### Execute CBS with Space-time A* and Disjoint Splitting
```shell
python run_experiments.py --instance "instances/test_*" --solver CBS --disjoint --batch
```

### Execute CBS with SIPP
```shell
python run_experiments.py --instance "instances/test_*" --solver CBS_SIPP --batch
```

### Execute CBS with SIPP and Disjoint Splitting
```shell
python run_experiments.py --instance "instances/test_*" --solver CBS_SIPP --disjoint --batch
```