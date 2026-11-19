# Nguồn gốc và tính toàn vẹn

Baseline lấy từ **R1 T03 P7 ngày 29/10/2026** và **V13 Coverfix Frozen ngày 28/10/2026** của project R1.

| Dữ liệu | Cách theo dõi |
|---|---|
| P7: mã, CAD, đường rãnh và quỹ đạo | `simulation/p7/PACKAGE_SHA256.json` |
| V13 runtime: controller, scene, mesh và mốc V10 | `reference/v13/CONTENT_MANIFEST.json` |
| Ánh xạ đường dẫn gốc sang repository, SHA trước/sau | [provenance.json](provenance.json) |
| Kết quả kiểm chứng bản bố trí mới | [baseline.json](../reports/baseline.json) |

Hai manifest của repository mô tả **bản tuyển chọn runtime**, không phải bản sao đủ file của các gói phân phối trước đây. README hướng dẫn, launcher Windows cũ, ảnh chụp giao diện và ZIP phát hành lặp đã được loại khỏi snapshot runtime.

Nguồn được giữ lại có mã băm so với bản gốc. Thay đổi duy nhất trong mã P7 nhập từ nguồn là vị trí tham chiếu và bộ kiểm manifest; các thuật toán hình học, dữ liệu CAD và chuyển động không bị thay đổi. Các file mới của repository gồm điểm vào độc lập đường dẫn máy, bộ kiểm V13, tài liệu và CI.

Các ghim nguồn nguyên bản vẫn được kiểm tra độc lập: XML V09 tích hợp, trace V10, ZIP tham chiếu V5.1.3 và quỹ đạo V13 phải khớp các SHA-256 trong hiệu chuẩn. Manifest P7 cũng ghim quỹ đạo P6 FIX1 mà nhánh P7 kế thừa.

Git giữ nguyên byte của snapshot trên các hệ điều hành để tránh thay đổi mã băm do chuẩn hóa xuống dòng.
