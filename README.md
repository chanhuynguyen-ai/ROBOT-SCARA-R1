# ROBOT SCARA R1

Mô phỏng robot **SCARA R1** tích hợp đầu kẹp **GF01 R02** để gắp, vận chuyển và đưa **G23** vào cụm bánh xe trong MuJoCo.

**Baseline:** P7 · **Tham chiếu:** V13 · **Quỹ đạo:** 2.622 frame / 13 pha

**Candidate test:** 

**Test 1: Điều khiển ROBOT SCARA** 

ứng viên tạo 1 gui bằng code python để điều khiển robot scara mô phỏng trên mujoco hoạt động tất cả các khớp

**Test 2: Điều khiển ROBOT tự động từ điểm bắt đầu đến điểm đặt**

ứng viên bổ sung môi trường vật lý cho mô phỏng (bổ sung lực kẹp và trọng lực)
ứng viên lựa chọn động cơ phù hợp tính toán và tích hợp sao cho phù hợp nhất với yêu cầu là tối ưu chi phí nhưng vẫn bảo đảm được hiệu năng


![SCARA R1 tích hợp GF01 và G23](docs/media/scara-r1-overview.png)


## Thành phần

| Thành phần | Vai trò |
|---|---|
| SCARA R1 | Cánh tay 4 khớp J1–J3–J2–J4 |
| GF01 R02 | Cơ cấu pitch và bốn cụm kẹp C gắn vào robot |
| G23 | Chi tiết gắp xe được nâng và đưa vào vị trí lắp |
| G08 / G56 | Bánh sau, trục tham chiếu và bánh trước trong cell |
| P7 | Nhánh CAD hai rãnh R13, kế thừa quỹ đạo P6 FIX1 |
