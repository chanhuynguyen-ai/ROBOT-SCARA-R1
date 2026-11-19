# Lịch sử baseline

Các số V và P thuộc những nhánh khác nhau; bảng sắp theo thời gian và vai trò kỹ thuật.

| Mốc | Nhánh | Nội dung chính |
|---|---|---|
| 19/10/2026 | R1 V5.1.3 | Khôi phục cell, nâng G23 và đi vòng bánh; còn dùng đầu kẹp proxy. |
| 23/10/2026 | GF01 U4C V23 / R03 | Pilot đầu kẹp riêng và bản sửa runner. |
| 27/10/2026 | R1 V10–V12 | Gắp/đặt với GF01 tích hợp; lập lại đường đi theo datum và gate nhả. |
| 27/10/2026 | V13 Calibration / VISFIX | Hiệu chuẩn bố trí mô phỏng, căn bệ và nền. |
| 28/10/2026 | V13 Coverfix | Điều chỉnh hiển thị vỏ và đầu kẹp; đóng băng nguồn tham chiếu. |
| 28/10/2026 | P6 FIX1 | Quỹ đạo dẫn G23 theo hai rãnh, làm mốc cho P7. |
| 29/10/2026 | P7 | Nới hai rãnh G23 lên R13, giữ quỹ đạo và datum; bổ sung kiểm hình học. |
| 19/11/2026 | Repository baseline | Tuyển chọn runtime, tách P7 và V13, chuẩn hóa manifest, tài liệu kỹ thuật và CI. |

## Phạm vi bản đưa lên repository

Repository theo dõi bản tích hợp P7 và các nguồn V13 cần để tái lập kết quả. Các gói phát hành trùng, môi trường Python cục bộ, launcher theo máy, bản chạy sinh ra và hồ sơ lịch sử không phục vụ runtime được giữ ngoài cây project.

Hình học, hiệu chuẩn, quỹ đạo P7 và các nguồn V13 giữ nguyên nội dung gốc. File `r1_p7_windows.py` được điều chỉnh để tìm nguồn theo cây repository và sử dụng manifest runtime mới. Chi tiết nằm trong [nguồn dữ liệu](PROVENANCE.md).
