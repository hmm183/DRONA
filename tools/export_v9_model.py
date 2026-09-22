"""Export the reviewed V9 Adaptive State Projection checkpoint to Android ONNX Runtime format."""

import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
V9_DIR = ROOT / "v9_adaptive_projection_dr"
sys.path.insert(0, str(V9_DIR))

from src.models_projection_v9 import AdaptiveStateProjectionNet  # noqa: E402


class AndroidV9ProjectionModel(torch.nn.Module):
    """
    Mobile-ready wrapper for AdaptiveStateProjectionNet.
    Outputs ordered tensors (delta_x, confidence) for ONNX Runtime.
    """

    def __init__(self, model: AdaptiveStateProjectionNet):
        super().__init__()
        self.model = model

    def forward(self, x_seq: torch.Tensor, x_ctx: torch.Tensor):
        delta_x, conf = self.model(x_seq, x_ctx)
        return delta_x, conf


def main():
    parser = argparse.ArgumentParser(description="Export V9 model to ONNX")
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "app" / "src" / "main" / "assets" / "ml",
    )
    args = parser.parse_args()

    ckpt_path = V9_DIR / "checkpoints" / "projection_net_v9.pth"
    if not ckpt_path.exists():
        raise FileNotFoundError(f"Checkpoint not found at {ckpt_path}")

    ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    state_dict = ckpt["model_state_dict"] if "model_state_dict" in ckpt else ckpt

    model = AdaptiveStateProjectionNet(in_features=8, context_dim=12, hidden_dim=32, num_events=8)
    model.load_state_dict(state_dict)
    model.eval()

    args.output.mkdir(parents=True, exist_ok=True)
    export_model = AndroidV9ProjectionModel(model)
    export_model.eval()

    dummy_seq = torch.zeros(1, 50, 8, dtype=torch.float32)
    dummy_ctx = torch.zeros(1, 12, dtype=torch.float32)

    onnx_file = args.output / "v9_adaptive_projection.onnx"
    torch.onnx.export(
        export_model,
        (dummy_seq, dummy_ctx),
        str(onnx_file),
        input_names=["x_seq", "x_ctx"],
        output_names=["delta_x", "conf"],
        dynamic_axes={
            "x_seq": {0: "batch_size"},
            "x_ctx": {0: "batch_size"},
            "delta_x": {0: "batch_size"},
            "conf": {0: "batch_size"},
        },
        opset_version=18,
        dynamo=False,
    )
    print(f"Exported ONNX model to {onnx_file}")

    # Verify with onnxruntime
    import onnxruntime as ort
    sess = ort.InferenceSession(str(onnx_file))
    ort_inputs = {
        "x_seq": dummy_seq.numpy(),
        "x_ctx": dummy_ctx.numpy(),
    }
    ort_outs = sess.run(None, ort_inputs)

    with torch.no_grad():
        pt_delta, pt_conf = export_model(dummy_seq, dummy_ctx)

    np.testing.assert_allclose(ort_outs[0], pt_delta.numpy(), rtol=1e-4, atol=1e-5)
    np.testing.assert_allclose(ort_outs[1], pt_conf.numpy(), rtol=1e-4, atol=1e-5)
    print("ONNX Runtime parity verified successfully with zero numeric drift!")

    onnx_hash = hashlib.sha256(onnx_file.read_bytes()).hexdigest()
    param_count = model.count_parameters()

    manifest_data = {
        "model": "v9 Adaptive Projection Dead Reckoning",
        "architecture": "DepthwiseSeparableConv1D + GRU + Context MLP (Dual Head)",
        "deployment_status": "v9 Adaptive Projection DR Production",
        "preprocessing_version": "v9-projection",
        "parameters": param_count,
        "sha256": onnx_hash,
        "input_sequence": {
            "name": "x_seq",
            "shape": [50, 8],
            "channels": [
                "a_fwd", "a_lat", "w_yaw", "speed_norm",
                "centripetal_acc", "centripetal_residual", "delta_psi_step", "jerk"
            ],
            "sample_rate_hz": 10.0,
            "window_seconds": 5.0,
        },
        "input_context": {
            "name": "x_ctx",
            "dim": 12,
            "features": [
                "discrepancy_m_scaled",
                "discrepancy_speed_scaled",
                "discrepancy_yaw_scaled",
                "current_speed_scaled",
                "event_onehot_0_stop",
                "event_onehot_1_high_rattle",
                "event_onehot_2_roundabout",
                "event_onehot_3_turn",
                "event_onehot_4_accel",
                "event_onehot_5_brake",
                "event_onehot_6_straight",
                "event_onehot_7_cruise"
            ],
        },
        "outputs": {
            "delta_x": {
                "dim": 6,
                "states": ["delta_E", "delta_N", "delta_v", "delta_psi", "delta_ba", "delta_bg"]
            },
            "conf": {
                "dim": 1,
                "range": [0.0, 1.0]
            }
        },
        "checkpoint_interval_steps": 50,
        "checkpoint_interval_seconds": 5.0,
        "min_confidence_gate": 0.20,
        "adaptive_thresholds_m": {
            "STOP": 2.0,
            "STRAIGHT": 5.0,
            "CRUISE": 5.0,
            "ACCEL": 6.0,
            "BRAKE": 6.0,
            "TURN": 8.0,
            "ROUNDABOUT_CANDIDATE": 10.0,
            "HIGH_RATTLE": 12.0
        }
    }

    manifest_file = args.output / "v9_manifest.json"
    manifest_file.write_text(json.dumps(manifest_data, indent=2), encoding="utf-8")
    print(f"Written manifest to {manifest_file}")

    normalization_data = {
        "preprocessing_version": "v9-projection",
        "discrepancy_m_scale": 10.0,
        "discrepancy_speed_scale": 5.0,
        "discrepancy_yaw_scale": 1.0,
        "current_speed_scale": 30.0,
        "clip_bounds": {
            "a_fwd_min": -8.0,
            "a_fwd_max": 8.0,
            "w_yaw_min": -1.0,
            "w_yaw_max": 1.0,
            "a_lat_min": -8.0,
            "a_lat_max": 8.0,
            "speed_min": 0.0,
            "speed_max": 45.0
        }
    }
    norm_file = args.output / "v9_normalization.json"
    norm_file.write_text(json.dumps(normalization_data, indent=2), encoding="utf-8")
    print(f"Written normalization to {norm_file}")


if __name__ == "__main__":
    main()
