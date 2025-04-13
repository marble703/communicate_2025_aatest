# 使用指南

![image](./image.png)

1. target

目标对象,只有 0 / 1
0 输出 yaw，1 输出 pitch

2. value

值，只在 wave = 0  时起效，控制输出值

3. wave

波形
0 -> 无波形
1 -> 方波
2 -> 锯齿波
3 -> 正弦波
4 -> 三角波
5 -> 随机波
6 -> 叠加正弦波

4. period

周期，单位100ms

5. ampitude

振幅，单位 0.01 弧度

6. offset

偏置，单位0.01弧度，从 0 - 628 ，实际是 -3.14 到 3.14

7. phase

8. noise_enable

是否启用噪声

9. noise_amplitude

噪声项强度,单位 0.01 弧度