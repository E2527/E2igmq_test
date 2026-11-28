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
- 電源を切っていても、スマホから起こせるようにもできます(下の 6)

### PC を起動したとき、WSL と Docker も動くようにする

SSH でつなぐと WSL が起動するので、ふつうは何もしなくて大丈夫です。`ros` で「Docker が動いていない」と言われたら:

```bash
sudo service docker start   # WSL の中で
```

(WSL で systemd を使っていれば、Docker は WSL の起動と一緒に動きます。`ps -p 1 -o comm=` で `systemd` と出れば使っています)

## 6. スマホから PC の電源を入れる(Wake-on-LAN)

**Wake-on-LAN** は、ネットワークで「起きて」という合図(マジック パケット)を送って、PC の電源を入れるしくみです。
有線 LAN の部品の **MAC アドレス**(部品ごとの名札。`A4-BB-6D-12-3F-9C` のような形)にあてて送ります。

- **有線 LAN** と **AC アダプター**をつないでおく(Wi-Fi だけでは、まず起きない)
- 合図は**家の中のネットワークにしか届かない**(Tailscale を通しては届かない。家の外からは下の「家の外から起こす」)

### PC の準備(PowerShell を「管理者として実行」で)

プロンプトが `C:\Windows\system32>` になっていれば管理者です。`C:\Users\...>` だと「アクセスが拒否されました」になります。

```powershell
# 1. 有線 LAN の部品が、PC を起こせる部品か(一覧に「Realtek ... GbE」「Intel ... Ethernet」などがあれば OK)
powercfg /devicequery wake_programmable

# 2. その部品で PC を起こせるようにする(名前は 1 の一覧のとおりに)
powercfg /deviceenablewake "Realtek PCIe GbE Family Controller"

# 3. 高速スタートアップを切る(電源を切った状態から起きるのに必要)
reg add "HKLM\SYSTEM\CurrentControlSet\Control\Session Manager\Power" /v HiberbootEnabled /t REG_DWORD /d 0 /f

# 4. 有線 LAN の設定。まず一覧を見る(「イーサネット」は Get-NetAdapter で出る有線 LAN の名前)
Get-NetAdapterAdvancedProperty -Name "イーサネット" | Format-Table DisplayName, DisplayValue, RegistryKeyword

# 5. 合図で起きる設定を有効、省エネ(LAN が切れて合図を受け取れなくなる)を無効にする
Set-NetAdapterAdvancedProperty -Name "イーサネット" -RegistryKeyword "*WakeOnMagicPacket" -RegistryValue 1
Set-NetAdapterAdvancedProperty -Name "イーサネット" -RegistryKeyword "S5WakeOnLan" -RegistryValue 1
Set-NetAdapterAdvancedProperty -Name "イーサネット" -RegistryKeyword "*EEE" -RegistryValue 0
Set-NetAdapterAdvancedProperty -Name "イーサネット" -RegistryKeyword "EnableGreenEthernet" -RegistryValue 0
Set-NetAdapterAdvancedProperty -Name "イーサネット" -RegistryKeyword "PowerSavingMode" -RegistryValue 0

# 6. 確かめる: 有線 LAN の部品が出れば、起こせる状態になっている
powercfg /devicequery wake_armed
```

- 5 で「見つからない」と出た行は、そのドライバーにない設定なので、気にしなくてよい(ドライバーによって名前がちがう。`RegistryKeyword` の列の名前で指定すると、言語に関係なく設定できる)
- スリープの種類は `powercfg /a` で分かる。「スタンバイ (S0 低電力アイドル)」(モダン スタンバイ)が使える PC は、`*ModernStandbyWoLMagicPacket` も 1 にする。「スタンバイ (S3)」だけなら不要
- 電源を切った状態から起きないときは、BIOS(起動するときにロゴの間に F2 か Del)で「Wake on LAN」「Power On by PCI-E」などを有効にする。BIOS に設定がない機種は、使わない間は**スリープ**にしておく

### 合図を送るアプリ(スマホ)の設定

Wake-on-LAN のアプリ(「Wake On Lan」などで検索)に、PC を登録する。PC の PowerShell で調べる。

| 項目 | 入れるもの | 調べ方 |
|---|---|---|
| MAC アドレス | 有線 LAN の部品の MAC アドレス | `Get-NetAdapter` の「イーサネット」の行の `MacAddress`(Wi-Fi の行ではない) |
| ブロードキャスト アドレス | 家のネットワークの番号の最後を 255 にしたもの(例 `192.168.1.255`) | `ipconfig` の「**イーサネット アダプター イーサネット**」の IPv4 アドレス(例 `192.168.1.7`)の最後を 255 に |
| ポート | `9`(だめなら `7`) | |
| ping アドレス(なくてもよい) | 有線 LAN の IPv4 アドレス(例 `192.168.1.7`) | 起きているかの表示に使うだけ。Windows はふだん ping に返事をしないので、表示も使うなら `Enable-NetFirewallRule -Name FPS-ICMP4-ERQ-In` |

- `ipconfig` の「vEthernet (WSL)」「Tailscale」「イーサネット 3」などは仮想の LAN なので使わない
- iPhone は **設定 → プライバシーとセキュリティ → ローカルネットワーク** でアプリをオンにする(オフだと「An error occurred when trying to send magic packet」)
- スマホは**家の Wi-Fi** につないで送る(モバイル回線からは届かない)
- 家のルーターが PC の IP アドレスを変えることがある。変わらないようにするには、ルーターの「DHCP の予約(固定割り当て)」で PC の有線 LAN を固定する

### 試す順番

1. PC を**スリープ**にして、スマホから合図を送る → 起きれば OK
2. PC を**シャットダウン**して、合図を送る → 起きれば、使わないときは電源を切っておける
3. 起きたら、スマホの SSH アプリで Tailscale のアドレスにつないで、`ros` でコンテナに入れるか確かめる

### 家の外から起こす

合図は家の中のネットワークにしか届かないので、家の外からは、**家にある「いつも電源が入っている機械」**(Raspberry Pi など)に頼んで送ってもらう。

```
スマホ(外)──Tailscale + SSH──▶ Raspberry Pi(家)── 合図 ──▶ PC が起きる
```

```bash
# Raspberry Pi に、最初の 1 回だけ(Tailscale も入れて、スマホと同じアカウントでログインしておく)
sudo apt install wakeonlan
# 起こすとき(スマホから Pi に SSH でつないで)
wakeonlan -i 192.168.1.255 A4:BB:6D:12:3F:9C     # ブロードキャスト アドレスと、PC の MAC アドレス
```

ルーターに Wake-on-LAN の機能があれば、それでもよい。

---

## 困ったとき

| 症状 | 原因と直し方 |
|---|---|
| つながらない(タイムアウト) | PC が寝ている / Tailscale がどちらかで切れている。スマホと PC の両方の Tailscale のアプリで「Connected」になっているか見る |
| `Permission denied` | ユーザー名がちがう / 鍵が登録されていない / `administrators_authorized_keys` の権限(`icacls` の行)をやり直す |
| Ubuntu ではなく PowerShell や cmd が出る | 2 の `DefaultShell` の行をやり直す |
| `ros: command not found` | WSL の `~/.bashrc` に alias がない([setup.md](setup.md) の「C. リポジトリとイメージの準備」の 4) |
| シミュレーターの画面を出そうとするとエラー | SSH では画面は出ない。画面が要るときは PC の前か、リモートデスクトップで |
| PowerShell で「アクセスが拒否されました」 | 管理者で開いていない。スタートメニューで PowerShell を右クリック →「管理者として実行」 |
| アプリで「An error occurred when trying to send magic packet」 | スマホが合図を送れていない。iPhone の「ローカルネットワーク」の許可、ブロードキャスト アドレス(`255.255.255.255` ではなく `192.168.1.255` のように)、家の Wi-Fi につながっているかを確かめる |
| 合図を送っても起きない | 有線 LAN と AC アダプター、MAC アドレスが有線 LAN のものか、6 の `wake_armed` に有線 LAN の部品が出るか。電源を切った状態から起きないなら BIOS の設定 |

## 気をつけること

- **実機のロボットを、遠くから動かさないでください。** 遠くからやるのは、シミュレーション・プログラムの編集・ビルド・テストだけにします。
- SSH はルーターで外に開けず、Tailscale の中だけで使います。
- パスワードではなく、鍵でつなぎます(4)。

## これから

- 画面を見る(Chrome リモートデスクトップ / RustDesk)
