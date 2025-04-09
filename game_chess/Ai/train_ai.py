from ultralytics import YOLO

# สร้างโมเดลใหม่จากไฟล์ YAML
model = YOLO("yolo12n.yaml")  # สร้างโมเดลใหม่จากไฟล์ YAML

# เพิ่มการฝึกโมเดล
if __name__ == '__main__':
    results = model.train(
        data="D:/RobotChess/Chess/train_Ai/Chess.v8i.yolov12/data.yaml",  # กำหนดไฟล์การตั้งค่าชุดข้อมูล
        epochs=100,  # จำนวน epoch สำหรับการฝึก
        imgsz=640,  # ขนาดภาพสำหรับการฝึก
        batch=16,  # batch size ที่เหมาะสมกับ RTX 3070
        device='cuda',  # ใช้ GPU (RTX 3070) สำหรับการฝึก
        workers=8,  # ใช้ 8 workers สำหรับการโหลดข้อมูล (ขึ้นอยู่กับ CPU)
        multi_scale=True,  # เปิดการฝึกหลายขนาดภาพ
        save=True,  # บันทึกโมเดลหลังการฝึก
        save_period=-1,  # บันทึกที่สิ้นสุดการฝึก
        rect=True,  # โหมดการฝึกแบบสี่เหลี่ยม (rectangular batch mode) เพื่อปรับแต่งขนาดภาพ
        warmup_epochs=3,  # ช่วง warm-up สำหรับ learning rate
        lr0=0.01,  # อัตราการเรียนรู้เริ่มต้น
        momentum=0.937,  # ค่าความเร็วการอัพเดท (momentum)
        weight_decay=0.0005,  # ป้องกัน overfitting โดยใช้ L2 regularization
        box=7.5,  # น้ำหนักสำหรับ box loss
        cls=0.5,  # น้ำหนักสำหรับ classification loss
        dfl=1.5,  # น้ำหนักสำหรับ distribution focal loss
        conf=0.25,  # ความเชื่อมั่นขั้นต่ำสำหรับการทำนาย
        plots=True  # เปิดการบันทึกกราฟการฝึกเพื่อการประเมินผล
    )
