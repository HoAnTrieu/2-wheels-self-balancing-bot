# Robot Cân Bằng Tự Động 2 Bánh Xe

Dự án robot cân bằng tự động sử dụng ESP32, con quay hồi chuyển MPU6050, module điều khiển động cơ L298N và thuật toán điều khiển PID. Robot cân bằng trên hai bánh xe như một con lắc ngược.

## Tổng Quan Dự Án

Dự án này triển khai robot hai bánh xe cân bằng tự động duy trì vị trí thẳng đứng thông qua điều khiển PID. Nó sử dụng kết hợp cảm biến từ MPU6050 để ước tính góc nghiêng và điều khiển hai động cơ DC để sửa chữa sự cân bằng.

Dự án bao gồm:
- Firmware Arduino ESP32 cho điều khiển cân bằng
- Scripts MATLAB cho mô hình hóa hệ thống và tính toán tham số
- Thiết kế khung in 3D
- Các thư viện cần thiết

## Yêu Cầu Phần Cứng

- Board phát triển ESP32 (ví dụ: ESP32 DevKit V1)
- Module con quay hồi chuyển MPU6050
- Module điều khiển động cơ L298N
- 2x Động cơ DC có bánh xe (3-6V, khuyến nghị 200RPM)
- 2x Pin 18650 hoặc nguồn điện phù hợp (7.4V)
- Khay pin và công tắc nguồn
- Dây jumper cho kết nối
- Khung in 3D (tệp STL từ Thingiverse)

## Yêu Cầu Phần Mềm

- Arduino IDE với hỗ trợ ESP32
- MATLAB (tùy chọn, cho phân tích hệ thống)
- Thư viện Adafruit MPU6050
- Thư viện Adafruit BusIO
- Thư viện Adafruit Unified Sensor

## Sơ Đồ Kết Nối

### ESP32 đến MPU6050 (I2C)
- ESP32 GPIO 21 (SDA) → MPU6050 SDA
- ESP32 GPIO 22 (SCL) → MPU6050 SCL
- ESP32 3.3V → MPU6050 VCC
- ESP32 GND → MPU6050 GND

### ESP32 đến L298N Điều Khiển Động Cơ
- ESP32 GPIO 25 → L298N ENA (PWM Động Cơ A)
- ESP32 GPIO 26 → L298N IN1 (Hướng Động Cơ A)
- ESP32 GPIO 27 → L298N IN2 (Hướng Động Cơ A)
- ESP32 GPIO 33 → L298N ENB (PWM Động Cơ B)
- ESP32 GPIO 16 → L298N IN3 (Hướng Động Cơ B)
- ESP32 GPIO 17 → L298N IN4 (Hướng Động Cơ B)
- ESP32 5V → L298N VCC (Nguồn logic)
- Nguồn ngoài 7.4V → L298N VS (Nguồn động cơ)
- ESP32 GND → L298N GND

### Kết Nối Động Cơ
- Động Cơ A → L298N OUT1/OUT2
- Động Cơ B → L298N OUT3/OUT4

## Cài Đặt và Thiết Lập

1. **Cài Đặt Arduino IDE và Hỗ Trợ ESP32**
   - Tải Arduino IDE từ arduino.cc
   - Thêm hỗ trợ board ESP32 qua Board Manager (tìm "ESP32")

2. **Cài Đặt Các Thư Viện Cần Thiết**
   - Mở Arduino IDE
   - Vào Sketch → Include Library → Manage Libraries
   - Cài đặt "Adafruit MPU6050", "Adafruit BusIO", "Adafruit Unified Sensor"

3. **Chuẩn Bị Phần Cứng**
   - Lắp ráp khung in 3D
   - Gắn động cơ, bánh xe và điện tử
   - Kết nối theo sơ đồ trên
   - Đảm bảo MPU6050 được gắn chắc chắn và ở giữa

4. **Tải Lên Code**
   - Mở một trong các sketch Arduino (ví dụ: PID-main.ino)
   - Chọn ESP32 Dev Module từ Tools → Board
   - Đặt tốc độ upload là 115200
   - Chọn cổng COM đúng
   - Nhấp Upload

5. **Bật Nguồn**
   - Kết nối nguồn đến L298N (nguồn động cơ)
   - Robot sẽ bắt đầu cân bằng
   - Giám sát đầu ra serial để debug

## Giải Thích Tệp Code

### PID-main.ino
Firmware cân bằng chính. Tính năng:
- Bộ lọc bù cho ước tính góc từ MPU6050
- Bộ điều khiển PID cho sửa chữa cân bằng
- Điều khiển động cơ với bù deadband
- Tắt an toàn khi nghiêng vượt quá 35 độ
- Đầu ra debug serial

Tham số chính cần điều chỉnh:
- Kp, Ki, Kd: Hệ số PID (bắt đầu với Kp=18, Ki=0.6, Kd=0.8)
- PWM_DEADZONE: PWM tối thiểu để vượt ma sát động cơ (130)
- PWM_SLEW: Thay đổi PWM tối đa mỗi chu kỳ cho điều khiển mượt (20)

### PID-balencing-lite.ino
Phiên bản đơn giản hóa với tính năng an toàn arm/disarm. Bao gồm:
- Tự động arm khi trong góc an toàn
- Tham số PID cố định
- Triển khai bộ lọc bù khác

### PID-balencing-test.ino
Firmware test cho xác nhận phần cứng. Chạy động cơ qua các chuỗi định sẵn trong khi giám sát cảm biến. Hữu ích cho:
- Test hướng động cơ
- Hiệu chuẩn đọc cảm biến
- Debug kết nối phần cứng

### MATLAB/ThongSoTWBR.m
Script MATLAB cho mô hình hóa hệ thống. Tính toán:
- Ma trận không gian trạng thái (A, B, C, D)
- Phân tích khả năng điều khiển và quan sát
- Hàm truyền
- Đồ thị pole-zero

Tham số bao gồm:
- Hằng số động cơ (km, ke, R)
- Kích thước vật lý (bán kính bánh xe r, khoảng cách l)
- Khối lượng và quán tính (Mp, Mw, Ip, Iw)

## Cách Code Hoạt Động

### Đọc Cảm Biến và Ước Tính Góc
MPU6050 cung cấp dữ liệu gia tốc kế và con quay hồi chuyển. Bộ lọc bù kết hợp:
- Gia tốc kế: Cung cấp góc nghiêng tuyệt đối (pitch/roll)
- Con quay hồi chuyển: Cung cấp tốc độ góc cho độ chính xác ngắn hạn

Công thức bộ lọc: `angle = α * accel_angle + (1-α) * (previous_angle + gyro_rate * dt)`

### Điều Khiển PID
Bộ điều khiển PID tính toán đầu ra động cơ dựa trên lỗi (góc mong muốn - góc hiện tại):
- Tỷ lệ (P): Sửa chữa ngay lập tức dựa trên lỗi hiện tại
- Tích phân (I): Sửa chữa lỗi trạng thái ổn định tích lũy
- Vi phân (D): Giảm dao động bằng cách dự đoán thay đổi lỗi

Output = Kp * error + Ki * ∫error dt + Kd * d(error)/dt

### Điều Khiển Động Cơ
L298N điều khiển hai động cơ độc lập:
- Chân hướng (IN1-IN4) đặt hướng quay
- Chân enable (ENA/ENB) đặt tốc độ qua PWM
- Bù deadband đảm bảo động cơ bắt đầu di chuyển ở tốc độ thấp

### Tính Năng An Toàn
- Tự động tắt khi nghiêng vượt quá giới hạn an toàn
- Giới hạn PWM và tốc độ slew ngăn chuyển động đột ngột
- Kẹp tích phân lỗi ngăn tích lũy

## Điều Chỉnh Bộ Điều Khiển PID

1. **Bắt Đầu Chỉ Với P**: Đặt Ki=0, Kd=0, tăng Kp cho đến khi dao động bắt đầu
2. **Thêm D**: Tăng Kd để giảm dao động
3. **Thêm I**: Ki nhỏ để loại bỏ lỗi trạng thái ổn định
4. **Điều Chỉnh Tinh**: Điều chỉnh cả ba tham số dần dần
5. **Test Độ Bền**: Đẩy robot nhẹ và quan sát phục hồi

Giám sát đầu ra serial cho giá trị góc và PWM trong quá trình điều chỉnh.

## Phân Tích Hệ Thống MATLAB

Chạy `ThongSoTWBR.m` trong MATLAB để phân tích hệ thống:
- Kiểm tra khả năng điều khiển (rank(P) nên là 4)
- Kiểm tra khả năng quan sát (rank(L) nên là 4)
- Kiểm tra vị trí pole cho độ ổn định
- Thiết kế bộ điều khiển sử dụng đồ thị root locus

## Khắc Phục Sự Cố

### Robot Ngã Ngay Lập Tức
- Kiểm tra hướng lắp MPU6050
- Xác nhận kết nối I2C
- Xác nhận hướng động cơ đúng
- Tăng hệ số P hoặc kiểm tra tính toán góc

### Động Cơ Không Chạy
- Kiểm tra kết nối nguồn L298N
- Xác nhận gán chân PWM và hướng
- Test với PID-balencing-test.ino

### Cân Bằng Không Ổn Định
- Điều chỉnh tham số PID
- Kiểm tra hiệu chuẩn cảm biến
- Đảm bảo đầu ra PWM mượt (điều chỉnh PWM_SLEW)

### Serial Hiển Thị "MPU6050 NOT FOUND"
- Kiểm tra kết nối I2C và điện trở kéo lên
- Xác nhận nguồn MPU6050 (3.3V)
- Test với sketch quét I2C

## Đóng Góp

Dự án này kết hợp code từ nhiều nguồn khác nhau. Vui lòng trích dẫn tác giả gốc khi sử dụng mô hình 3D hoặc đoạn code.

## Giấy Phép

Xem tệp LICENSE trong thư mục con cho thông tin giấy phép cụ thể.