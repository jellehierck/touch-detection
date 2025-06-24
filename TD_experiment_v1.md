<!-- LTeX: enabled=false -->

# TD exeriments v1

This document explains how I performed the first variation of the TD experiments.

## Conditions

These are the conditions tested.

| Exp nr | Sync type    | Offset |
| ------ | ------------ | ------ |
| 1      | timesyncd    | 0s     |
| 2      | chrony_local | 0s     |
| 3      | chrony_local | 15ms   |
| 4      | chrony_local | 45ms   |
| 5      | chrony_local | 150ms  |
| 6      | chrony_local | 1s     |

## Procedure

### 1. Set up clock synchronization on each computer and wait for synchronzation to settle

```bash
# AI PC
/home/nakama6000/Documents/git/clock_sync_ws/src/touch-detection/TD_experiment.sh --pc ai --setup-sync -e 1

# Robot PC
/home/fr3/nakama_ws/src/touch-detection/TD_experiment.sh --pc robot --setup-sync -e 1

# Force PC
/home/jelle/thesis/nakama_ws/src/touch-detection/TD_experiment.sh --pc force --setup-sync -e 1
```

### 2. Set up experiment nodes for clock monitoring and PC-specific nodes

```bash
# Robot PC
/home/fr3/nakama_ws/src/touch-detection/TD_experiment.sh --pc robot --setup-exp  -e 1

# Force PC
/home/jelle/thesis/nakama_ws/src/touch-detection/TD_experiment.sh --pc force --setup-exp -e 1 -r 1 
```

### 3. Start the repetition only on the Force PC

```bash
# Force PC
/home/jelle/thesis/nakama_ws/src/touch-detection/TD_experiment.sh --pc force --start-rep -e 1 -r 1 
```

Optionally, delete repetition data first:

```bash
# Force PC
/home/jelle/thesis/nakama_ws/src/touch-detection/TD_experiment.sh --pc force --empty-rep -e 1 -r 1 

# Force PC remove folders with incorrect permissions
/home/jelle/thesis/nakama_ws/src/touch-detection/TD_experiment.sh --pc force --force-empty-rep -e 1 -r 1 
```

### 4. After all repetitions, clean up experiments

```bash
# Robot PC
/home/fr3/nakama_ws/src/touch-detection/TD_experiment.sh --pc robot --clean-exp -e 1

# Force PC
/home/jelle/thesis/nakama_ws/src/touch-detection/TD_experiment.sh --pc force --clean-exp -e 1
```

### 5. After all experiments are done, restore synchronization

```bash
# AI PC
/home/nakama6000/Documents/git/clock_sync_ws/src/touch-detection/TD_experiment.sh --pc ai --clean-sync  -e 1 

# Robot PC
/home/fr3/nakama_ws/src/touch-detection/TD_experiment.sh --pc robot --clean-sync  -e 1

# Force PC
/home/jelle/thesis/nakama_ws/src/touch-detection/TD_experiment.sh --pc force --clean-sync -e 1
```

## Experiment notes
