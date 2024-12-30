# ML-Agents Training Setup

## Prerequisites
- Unity Hub and Editor installed
- Python 3.10.11 installed
- ML-Agents package installed in Unity project

## Setup Steps

1. Install ML-Agents Python package:
```bash
pip install mlagents
```

## Running Training

1. Open terminal in project root directory

2. Start ML-Agents training:
```bash
mlagents-learn Agents/config.yaml --run-id=training1
```

3. Press Play in Unity Editor when prompted

4. Start TensorBoard in new terminal:
```bash
tensorboard --logdir results --port 6006
```

5. View metrics:
   - Open browser to `localhost:6006`
   - Enable "Show data download links" in upper left
   - Download CSV data using buttons under graphs

## Collecting Results

1. Let training run until performance stabilizes

2. In TensorBoard:
   - Take screenshots of key metric graphs:
     - Cumulative Reward
     - Episode Length
     - Policy Loss
     - Success Rate
     - Correct/Incorrect Hits

3. Submit:
   - Word doc with graph screenshots and analysis
   - ZIP file with CSV data from TensorBoard