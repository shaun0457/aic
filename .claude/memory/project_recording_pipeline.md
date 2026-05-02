---
name: Recording Pipeline Status
description: CheatCodeRecorder 錄製完成、格式轉換完成、ACT 訓練完成、RunACT Docker 測試通過
type: project
originSessionId: b7ee721c-794f-4b55-a14a-66fb01982e34
---
## 當前進度（截至 2026-05-02）

### ✅ 完成步驟

1. **111 episodes 錄製完成**
   - 原始資料：`~/ws_aic/src/aic/aic_dataset_partial/`（8.7GB）
   - HuggingFace：`shaun0457/aic_cheatcode_demos`（tar 格式）

2. **lerobot 格式轉換完成**
   - 輸出：`~/.cache/huggingface/lerobot/shaun0457/aic_cheatcode_demos/`（7.3GB）
   - 110 episodes（episode_000110 無 meta.json 跳過）
   - 指令：`python3 scripts/convert_to_lerobot.py --src aic_dataset_partial --repo_id shaun0457/aic_cheatcode_demos`

3. **ACT 訓練完成（GCP VM Tesla T4）**
   - Model：`shaun0457/act_aic`（HuggingFace，已自動上傳）
   - 100K steps，final loss ~0.065，約 6.5 小時

4. **RunACT Docker image build 完成**
   - Image：`my-solution:v1`（GCP VM 本地）
   - Dockerfile 關鍵修改：
     - `pixi install --frozen`（原 `--locked` 會失敗，lock file 是 macOS 生成的）
     - source override：cp aic_example_policies 到 site-packages（pixi 用快取舊版本）
     - 預下載 `shaun0457/act_aic` 權重
     - 預下載 ResNet18 backbone（torchvision ImageNet weights，ACTPolicy init 時需要）
     - `ENV HF_HUB_OFFLINE=1`

5. **RunACT 測試通過（GCP eval container）**
   - 跑法：`docker run --network container:aic-eval-1 my-solution:v1 ...RunACT`
   - 3 trials 全部完成，總分 **107.96 / ~300**（Trial 1: 39.8, Trial 2: 42.8, Trial 3: 25.4）

### RunACT.py 關鍵設計

- **module-level imports 為空**（只有 `_LightPolicy` 和 `RunACT` class 定義）
  - Why: `from aic_model.policy import Policy` 需要 15+ 秒，會導致 engine GetState timeout
- **重 imports 全部在 `__init__` 內**（torch、lerobot、draccus 等）
- **model path hardcoded**（避免 snapshot_download 網路請求）：
  `/root/.cache/huggingface/hub/models--shaun0457--act_aic/snapshots/25f6c067ea29d50b1b94221aff0c1fd7e10518fa`
- `_LightPolicy` stub 提供 `get_logger()` 和 `get_clock()`
- `time_limit = getattr(task, 'time_limit', 150)` — 不 hardcode，用 task 物件的時間限制

---

## 下一步

- [ ] git push（需 GitHub PAT 或 SSH key，code commit b81bc87 已在本地）
- [ ] docker save my-solution:v1 為 tar，或 push 到 Docker Hub（GCP 太貴，需搬遷）
- [ ] rebuild Docker image（RunACT.py time_limit 已改，需重 build）
- [ ] 繼續優化分數（現在 107.96/~300 = ~36%）
- [ ] 拿 Intrinsic auth token，提交到 registry（截止 2026/05/15）
