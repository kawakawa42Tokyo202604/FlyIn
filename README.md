# Fly-in

## English

A Python simulator that schedules multiple drones from a shared start to a destination. It reads a map of connected locations and prints where each drone moves over time.

Locations and connections limit how many drones can use them at once. Routes account for blocked locations and longer travel times. Time advances in turns, with one output line per turn.

### Run

Requires Python 3.10 or newer. No additional runtime packages are needed.

```sh
make
make run                 # Use config.txt
make run MAP=other.txt    # Use another map
python3 main.py config.txt
```

A map specifies drone count, start and destination coordinates, other locations, and connections. Output lists the drones moving each turn and their destinations.

### Small map example

Save this as `simple-map.txt` to send two drones directly from `a` to `b`:

```text
nb_drones: 2
start_hub: a 0 0
end_hub: b 1 0
connection: a-b
```

`make run MAP=simple-map.txt` prints the following, without showing terminal colors here. A connection has capacity 1 by default, so drone 1 arrives on turn 1 and drone 2 on turn 2.

```text
D1-b
D2-b
```

### Tests

```sh
make test
make lint  # Requires flake8 and mypy
```

Checks one or multiple drones, connection capacities, blocked locations, travel-time differences, malformed maps, and invalid arguments.

### Implementation

- Parses the map as a graph and checks reachability.
- Searches states of time and location, reserving routes for successive drones.
- Supports `normal`, `restricted`, `priority`, and `blocked` zones.

## 日本語

複数のドローンを、同じ出発地点から目的地へ運ぶ手順を計算するPythonのシミュレーターです。地点のつながりをマップとして読み込み、各ドローンがどの時間にどこへ移動するかを表示します。

1つの地点や経路を同時に使える台数には上限があります。通行禁止の地点や、通過に時間がかかる地点も考慮して、移動の順番と経路を決めます。時間は「ターン」という単位で進み、出力の1行が1ターンに対応します。

### 実行

Python 3.10以降で動作し、実行に追加ライブラリは不要です。

```sh
make
make run                 # config.txt を使用
make run MAP=other.txt   # 別のマップを指定
# または
python3 main.py config.txt
```

マップにはドローン数、出発地点・目的地、各地点の座標と接続を記述します。実行すると、各ターンに移動するドローンとその行き先を表示します。

### 小さなマップの例

次の内容を `simple-map.txt` に保存すると、2台を地点 `a` から `b` へ直接移動させるマップになります。

```text
nb_drones: 2
start_hub: a 0 0
end_hub: b 1 0
connection: a-b
```

`make run MAP=simple-map.txt` の表示は次のとおりです（文字色を除く）。経路の既定の容量は1台なので、1ターン目にドローン1、2ターン目にドローン2が到着します。

```text
D1-b
D2-b
```

### テスト

```sh
make test
make lint  # flake8・mypyが必要
```

1台・複数台の移動、経路の容量制限、通行禁止区域、移動時間が異なる区域、不正なマップや引数を確認します。

### 実装メモ

- マップをグラフとして読み込み、到達可能性を検証します。
- 時間と場所を状態として経路を探索し、各ドローンの移動を順に予約します。
- `normal`・`restricted`・`priority`・`blocked` の区域を扱います。
