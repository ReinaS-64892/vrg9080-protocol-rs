# vrg9080-protocol

VRG9080 の IMU・近接値を取得する非公式 Rust クレートです。機器ベンダーとの提携・承認はなく、限定した相互運用のための実装です。完全なドライバーではありません。

- 純粋なパーサーと、Linux `hidraw` の同期 API。async runtime は不要です。
- 対応機器は **USB VID `0x2C30` / PID `0x1050`** のみ。sysfs の `HID_ID` と、開いた FD の識別情報で検証します。
- 加速度（g）、角速度（deg/s）、磁気の生値、タイムスタンプ、温度、近接の生値を返します。軸順は変更せず、校正・姿勢推定はしません。
- 64 バイトの入力、version、length、SUM8、浮動小数点値を検証。識別情報を含むコマンド応答のペイロードは公開しません。

## 使用例

```toml
[dependencies]
vrg9080-protocol = { path = "/path/to/vrg9080-protocol-rs" }
```

```rust,no_run
use std::time::Duration;
use vrg9080_protocol::{DeviceSession, Event, TransportError};

# fn main() -> Result<(), Box<dyn std::error::Error>> {
let mut session = DeviceSession::connect()?;
// 複数の候補がある場合は DeviceSession::open("/dev/hidrawN") で指定。
session.request_current_proximity()?;
loop {
    match session.next_event(Duration::from_secs(1)) {
        Ok(Some(Event::Imu(frame))) => println!("{:?}", frame.acceleration_g),
        Ok(Some(Event::Status(update))) => {
            if let Some(value) = update.proximity {
                println!("proximity={value}");
            }
        }
        Ok(Some(Event::CurrentProximity(response))) => {
            println!("current={:?}, data={:?}, status={}",
                response.proximity, response.uninterpreted_data, response.status);
        }
        Ok(_) => {} // タイムアウト、その他の応答
        Err(TransportError::InvalidReport(_)) => continue,
        Err(error) => return Err(error.into()),
    }
}
# }
```

`parse_report(&[u8])`、`parse_imu_report(&[u8])`、`parse_status_report(&[u8])` は I/O を行いません。パーサーのみの利用では `default-features = false` にすると外部依存はなく、Linux 以外でも利用できます。入力は report-ID を含まない正確に 64 バイトです。

`timestamp_ticks` は公称 100 µs/tick の `u16` で周回します。差分計算は行いません。温度は符号なしの 0.1 ℃ 単位です。磁気と近接の値には未確認の物理単位や真偽の意味を付けません。ステータスの未選択フィールドは `None` です。

## 接続と書き込み

接続時に最大 500 ms 受信し、有効な IMU が届けば初期化要求を送りません。届かなければ下記の固定 15 段階を順に実行します。各要求は一度だけ送り、対応する成功応答を待ちます。途中で IMU が届き始めても最後まで実行し、失敗やタイムアウト時の自動再試行はしません。

| 段階 | コマンド | 固定ペイロード |
|---:|---:|---|
| 1 | `8C` | `00` |
| 2 | `8D` | `03` |
| 3 | `80` | `00 00` |
| 4 | `01` | なし |
| 5–7 | `7F` | 順に `01`、`00`、`02` |
| 8–10 | `02`、`40`、`47` | なし |
| 11 | `86` | `01 01` |
| 12 | `4A` | なし（初期近接問い合わせ） |
| 13–14 | `4E`、`4F` | なし |
| 15 | `81` | `E8 03 C8 00` |

応答待ちは段階 2 が 3.5 秒、その他は 1 秒です。書き込み可能になるまでの待機は最大 1 秒です。ただし OS 内部の USB 転送そのものが待つ時間はこのタイムアウトでは制御できず、OS 側のタイムアウトになる場合があります。初期化は一時的な動作状態とサンプリングレートを変更し、段階 15 の固定値は `(1000, 200)` です。利用者が変更する API はありません。

追加の書き込みは、明示的な `request_current_proximity()` による固定 `4A` 問い合わせと、検証済みステータスで ACK ビットが立っていた場合の応答だけです。通常の近接変化は非同期に届くステータスを同期 API で受信します。問い合わせは前の応答を受け取ってから再実行してください。汎用コマンド・任意ペイロード・生の書き込み API はありません。

`4A` 応答には、資料の長さ 6 に加えて、実機で長さ 7 が確認されています。長さ 6 の成功応答は `proximity: Some(u8)` を返します。長さ 7 では状態バイトを末尾で検証し、データ 2 バイトを `uninterpreted_data` に元の順序で保持します。手を離した状態と覆った状態の問い合わせはともに `[0, 0]` で、その間の非同期近接通知は `1/0` に変化しました。この 2 バイトの意味は未確認なので現在値を推定せず、`proximity` は `None` です。非同期ステータスの近接更新は資料どおりの 1 バイト値として取得できます。

接続中のイベントは最大 256 件保持します。満杯なら古い IMU を破棄し、`statistics().dropped_imu_reports` に計上します。ステータスや応答は保持し、それだけで満杯になった場合はエラーにします。不正レポートは接続中には計上して読み飛ばし、接続後には回復可能な `InvalidReport` を返します。`next_event(Duration::ZERO)` は待機せず確認します。

**セッションを破棄しても停止・電源断コマンドは送りません。** 別のドライバーによる状態変更や機器の切断まで配信が続くことがあります。同じ機器に対する他のドライバー・リーダーとの同時アクセスは未対応です。権限変更、sudo、カーネルドライバーの切り離し、他プロセスの終了は行いません。必要な `/dev/hidrawN` の読み書き権限は管理者に確認してください。

## 実行と検証

```sh
cargo run --example read_sensors -- --list
cargo run --example read_sensors -- --seconds 30 --query-proximity
cargo run --example read_sensors -- --device /dev/hidrawN --seconds 300
cargo test --all-features
cargo test --no-default-features
cargo clippy --all-targets --all-features -- -D warnings
```

CLI は IMU を既定で 100 件ごと（最初の 1 件も表示）、近接変化をその都度表示し、終了時に受信数・不正レポート数を表示します。`--imu-every 1` で全 IMU を表示できます。長さ 7 の `4A` 応答は未解釈の 2 バイト値として表示します。受信レポート全体や識別用ペイロードは出力しません。

実機テストは既定で無効です。他のリーダーが動作していない状態で明示的に実行します。

```sh
cargo test --test hardware -- --ignored --nocapture
# 対象を指定する場合:
VRG9080_HIDRAW=/dev/hidrawN cargo test --test hardware -- --ignored --nocapture
```

実機での基準確認は、USB を一度抜き差ししてから 30 秒以上の IMU 受信・近接変化を確認し、続けてプロセスを再実行して `startup_performed=false` を確認してください。`--seconds 300` で長時間受信も確認できます。

資料では段階 1 が IMU 配信開始の契機とされていますが、実機では段階 1 だけで IMU は届きませんでした。先頭 2 段階も検証できるようにしています。現時点では全手順を既定としています。比較には毎回 USB の抜き差しを行い、他の初期化処理の影響を除いた状態で IMU・近接・長時間動作を確認する必要があります。

実機で全手順の冷間起動後に 30 秒間、約 28,000 件の IMU と近接値 `1/0` の更新を受信し、要求された ACK の送信も確認しました。一方、1 段階だけの冷間起動では応答待ちタイムアウトが一度発生し、別の試行では応答に成功して近接更新が届いても 30 秒間 IMU が 0 件でした。1 段階版は IMU の初期化に不足しています。

先頭 2 段階では冷間起動後の 30 秒間に約 28,800～28,900 件の IMU を 2 回受信しました。近接更新は一方で 4 件、もう一方ではセンサーを覆って離しても 0 件でした。再実行時も IMU は継続しましたが、短縮版の近接動作は安定したと判断できないため、既定は全手順です。

短縮候補を明示的に試す場合は `StartupMode::StreamOnly`（先頭 1 段階）または `StartupMode::FirstTwo`（先頭 2 段階）を `DeviceSession::connect_with_startup()` または `open_with_startup()` に渡します。対応する成功応答を待ちます。サンプリングレートの設定と初期近接問い合わせは省略し、自動フォールバックや再試行は行いません。ACK の条件は同じです。

```sh
cargo run --release --example read_sensors -- --startup first-two --seconds 30
```

冷間起動での比較結果として有効なのは `startup_performed=true` の実行です。近接の現在値も確認したい場合だけ `--query-proximity` を加えます。受信した不正レポートの診断は、エラーとヘッダーのフラグ・種類・長さに限定し、ペイロードは含めません。

ライセンスは未設定、公開配布は無効（`publish = false`）です。
