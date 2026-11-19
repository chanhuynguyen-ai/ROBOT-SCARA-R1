# ROBOT SCARA R1

Mô phỏng robot **SCARA R1** tích hợp đầu kẹp **GF01 R02** để gắp, vận chuyển và đưa **G23** vào cụm bánh xe trong MuJoCo.

**Baseline:** P7 · **Tham chiếu:** V13 · **Quỹ đạo:** 2.622 frame / 13 pha · **Phạm vi:** nghiên cứu động học và hình học.

![SCARA R1 tích hợp GF01 và G23](docs/media/scara-r1-overview.png)


## Thành phần

| Thành phần | Vai trò |
|---|---|
| SCARA R1 | Cánh tay 4 khớp J1–J3–J2–J4 |
| GF01 R02 | Cơ cấu pitch và bốn cụm kẹp C gắn vào robot |
| G23 | Chi tiết gắp xe được nâng và đưa vào vị trí lắp |
| G08 / G56 | Bánh sau, trục tham chiếu và bánh trước trong cell |
| P7 | Nhánh CAD hai rãnh R13, kế thừa quỹ đạo P6 FIX1 |
