# スマホから PC につないで作業する(Tailscale + SSH)

家の外からでも、スマホで PC(Windows + WSL)のコンテナに入って、コマンドで作業する手順です。

```
スマホ(SSH アプリ)──Tailscale(安全な道)──▶ Windows の SSH ──▶ WSL(Ubuntu)──▶ ros(コンテナ)
```

- **Tailscale** … PC とスマホを、家の外からでも同じネットワークにいるようにする VPN。個人なら無料(3 人・100 台まで。使う前に料金のページで確認する)
- **SSH** … ほかの機械にコマンドを打つしくみ。Windows Home にも入っている

コマンドの作業(ビルド、テスト、git、ファイルの編集)はこれでできます。シミュレーターの画面は SSH では見えないので、画面が要るときは、あとで Chrome リモートデスクトップなどを足します。

**最初の 1 回だけ**やります(30 分くらい)。Windows Home を想定しています(Pro でも同じ)。

---

## 1. Tailscale を入れる

1. PC(Windows)に Tailscale を入れる: https://tailscale.com/download から Windows 版を入れて、Google などのアカウントでログイン(**WSL の中ではなく Windows に入れる**)
2. スマホにも Tailscale のアプリを入れて、**同じアカウント**でログイン
3. スマホの Tailscale のアプリに PC の名前が出て、「100.」で始まるアドレス(例 `100.101.102.103`)が付いていれば OK。このアドレスを、あとで使う

## 2. Windows で SSH を受け付ける

**PowerShell を「管理者として実行」で開いて:**

```powershell
# OpenSSH サーバーを入れる(設定 → システム → オプション機能 → 「OpenSSH サーバー」を追加 でも同じ)
Add-WindowsCapability -Online -Name OpenSSH.Server~~~~0.0.1.0

# 起動して、PC を起動したときに自動で起動するようにする
Start-Service sshd
Set-Service -Name sshd -StartupType Automatic

# SSH でつないだら、そのまま WSL(Ubuntu)に入るようにする
New-ItemProperty -Path "HKLM:\SOFTWARE\OpenSSH" -Name DefaultShell -Value "C:\Windows\System32\wsl.exe" -PropertyType String -Force

# Tailscale からだけ受け付ける(家や学校の Wi-Fi の、ほかの人からはつなげないようにする)
Set-NetFirewallRule -Name OpenSSH-Server-In-TCP -RemoteAddress 100.64.0.0/10
```

**ルーターのポートは開けません**(SSH をインターネットにそのまま出さない)。Tailscale の中だけで使います。

## 3. スマホからつないでみる(まずはパスワードで)

1. スマホに SSH のアプリを入れる(例: **Termius**。iPhone でも Android でも使える)
2. 新しい接続(Host)を作る
   - アドレス: 1 で調べた `100.` のアドレス
   - ユーザー名: Windows のユーザー名(PowerShell で `whoami` と打つと `PC名\ユーザー名` と出る。その後ろの部分)
   - パスワード: Windows にログインするときのパスワード(Microsoft アカウントなら、そのパスワード。PIN ではない)
3. つないで、`root@...` ではなく **Ubuntu のプロンプト**(`ユーザー名@PC名:~$`)が出れば成功
4. コンテナに入る:

```bash
ros                 # 開発用のコンテナに入る(setup.md で登録した alias)
```

## 4. パスワードの代わりに鍵でつなぐ(おすすめ)

パスワードは盗まれたり当てられたりするので、**鍵**(スマホだけが持っている秘密のファイル)でつなぐようにします。

1. Termius の Keychain → **Generate Key** で鍵を作る(種類は ED25519)
2. できた鍵の **公開鍵**(`ssh-ed25519 AAAA...` で始まる 1 行)をコピーする
3. PC の PowerShell(管理者)で、その 1 行を登録する(Windows の管理者のユーザーは、この場所に書く):

```powershell
Add-Content -Path C:\ProgramData\ssh\administrators_authorized_keys -Value "ssh-ed25519 AAAA...(コピーした 1 行)"
icacls C:\ProgramData\ssh\administrators_authorized_keys /inheritance:r /grant "Administrators:F" /grant "SYSTEM:F"
```

4. スマホの接続の設定で、パスワードの代わりにその鍵を選んで、つながるか確かめる
5. 鍵でつながったら、**パスワードでのログインを止める**:
   `C:\ProgramData\ssh\sshd_config` をメモ帳(管理者)で開き、`#PasswordAuthentication yes` の行を `PasswordAuthentication no` に書き換えて保存し、PowerShell(管理者)で `Restart-Service sshd`

- 鍵は、スマホをなくしたら使えなくなります。そのときは、PC で `administrators_authorized_keys` からその行を消します。
- **公開鍵**(`ssh-ed25519 AAAA...`)は人に見られても大丈夫ですが、**秘密鍵**はスマホの外に出さないでください。

## 5. 便利にする

### つないでいる途中で切れても、作業を続ける(tmux)

スマホの電波が切れると、そのとき動いていたコマンドも止まります。**tmux** の中で作業すると、切れても続きます。

```bash
sudo apt install tmux       # WSL で、最初の 1 回だけ
tmux new -A -s robo         # 作業を始めるとき(切れたあと、もう一度打つと続きにもどる)
```

### PC がスリープしないようにする

PC が寝ていると、つなげません。

- 設定 → システム → 電源 → 「電源に接続時、次の時間が経過した後にデバイスをスリープ状態にする」を **なし** にする
- ノート PC は、**AC アダプターをつないで**おく
- 遠くから電源を入れる(Wake-on-LAN)のは、BIOS の設定しだいです(下の「これから」)

### PC を起動したとき、WSL と Docker も動くようにする

SSH でつなぐと WSL が起動するので、ふつうは何もしなくて大丈夫です。`ros` で「Docker が動いていない」と言われたら:

```bash
sudo service docker start   # WSL の中で
```

(WSL で systemd を使っていれば、Docker は WSL の起動と一緒に動きます。`ps -p 1 -o comm=` で `systemd` と出れば使っています)

---

## 困ったとき

| 症状 | 原因と直し方 |
|---|---|
| つながらない(タイムアウト) | PC が寝ている / Tailscale がどちらかで切れている。スマホと PC の両方の Tailscale のアプリで「Connected」になっているか見る |
| `Permission denied` | ユーザー名がちがう / 鍵が登録されていない / `administrators_authorized_keys` の権限(`icacls` の行)をやり直す |
| Ubuntu ではなく PowerShell や cmd が出る | 2 の `DefaultShell` の行をやり直す |
| `ros: command not found` | WSL の `~/.bashrc` に alias がない([setup.md](setup.md) の「C. リポジトリとイメージの準備」の 4) |
| シミュレーターの画面を出そうとするとエラー | SSH では画面は出ない。画面が要るときは PC の前か、リモートデスクトップで |

## 気をつけること

- **実機のロボットを、遠くから動かさないでください。** 遠くからやるのは、シミュレーション・プログラムの編集・ビルド・テストだけにします。
- SSH はルーターで外に開けず、Tailscale の中だけで使います。
- パスワードではなく、鍵でつなぎます(4)。

## これから

- 遠くから PC の電源を入れる(Wake-on-LAN): BIOS の設定・有線 LAN・Raspberry Pi から合図を送る
- 画面を見る(Chrome リモートデスクトップ / RustDesk)
