# racecar for adam team
## Folders
### real
The scripts in this folder means that it worked on the real racecar.

### sim
The scripts in this folder means that it worked in sim, and haven't testes on the real racecar.

## scripts
### real
main_real.py => the main program that has been tested on the real racecar.

### sim
cone_slalom.py => have tested on sim and works. HSV is not adjusted for real racecar.

curve02.py => wallfollow + turn left, only right side.

wall_follow.py => wallfollow + turn left, both sides.

main_3.py => main program for sim.

wall_follow4.py => right angle curve

# memos
## cone HSV
((74,98,0),(118,255,255))

## cone regression
y = 2174.2984490852577 / x**0.5 + 0.07838245131948973

## line hsv
### 6th floor
((68,49,131),(100,184,255))

# ideas

## 蛇行の改善案

### angleが小さい場合に無視をする
```python
if angle > -0.05 and angle < 0.05:
    angle = 0
```
### angleの1updateあたりの可動域を設定する
```python
if abs(prev_angle - angle) > 0.1:
    if prev_angle - angle < 0:
        angle = prev_angle - 0.1
    else:
        angle = prev_angle + 0.1
```
### 過去のangleとの平均を取る
```python
angle = (prev_angle + angle) / 2
```
### angleの加重平均をとる
```python
angle = 0.7 * angle + 0.3 * prev_angle
```
### angleの非線形マッピングをする
```python
angle = np.tanh(angle * gain_factor)
```
## PID classの改善案

### 積分のWindup対策

積分項が無制限に増え続けると、制御出力が大きくなりすぎてしまう可能性があります。これを防ぐため、積分項に上限と下限を設ける必要があります。
### 微分項の雑音対策

微分項は雑音に弱いため、何らかの雑音除去フィルタを適用することが望ましいです。
### 初期化時の値設定

previous_errorとintegralの初期値をゼロに設定していますが、実際の使用時にはより適切な値を設定する必要があるかもしれません。
### dtの設定

dtは制御周期に依存するため、コンストラクタで固定値を設定するのではなく、updateメソッドの引数で渡す方が柔軟性が高くなります。

### 改良後案:
```python
class PID:
    def __init__(self, Kp, Ki, Kd, integral_max=100, integral_min=-100):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0
        self.integral_max = integral_max
        self.integral_min = integral_min

    def update(self, error, dt):
        # 積分項の計算と制限
        self.integral += error * dt
        self.integral = max(self.integral_min, min(self.integral_max, self.integral))

        # 微分項の計算 (ローパスフィルタ付き)
        derivative = (error - self.previous_error) / dt
        self.previous_error = error

        # 出力の計算
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        return output
```
