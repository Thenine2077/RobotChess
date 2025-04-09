from ultralytics import YOLO

# Load the pre-trained model (if you have a previous checkpoint, it can be loaded here)
model = YOLO("yolo12n.yaml")  # build a new model from YAML
model.load("D:/Project RobotChess/RobotChess/game_chess/runs/detect/train/weights/best.pt")  # load pre-trained model weights

# Training with the updated configuration
if __name__ == '__main__':
    results = model.train(
        data="D:/RobotChess/Chess/train_Ai/Chess.v8i.yolov12/data.yaml",               # Path to your dataset YAML
        epochs=100,                     # Number of epochs to continue training
        imgsz=640,                      # Image size used for training
        batch=16,                       # Batch size
        lr0=0.01,                       # Initial learning rate
        lrf=0.01,                       # Final learning rate (scaled from lr0)
        momentum=0.937,                 # Momentum for optimizer
        weight_decay=0.0005,            # Weight decay for regularization
        warmup_epochs=3.0,              # Warm-up epochs for learning rate
        warmup_momentum=0.8,            # Warm-up momentum
        warmup_bias_lr=0.1,             # Learning rate for bias parameters during warmup
        box=7.5,                        # Weight for box loss
        cls=0.5,                        # Weight for classification loss
        dfl=1.5,                        # Distribution focal loss weight
        pose=12.0,                      # Pose loss weight (for pose estimation tasks)
        kobj=1.0,                       # Keypoint objectness loss weight
        nbs=64,                         # Nominal batch size for normalization of loss
        multi_scale=True,               # Enable multi-scale training
        hsv_h=0.015,                    # Hue jitter
        hsv_s=0.7,                      # Saturation jitter
        hsv_v=0.4,                      # Value jitter
        degrees=0.0,                    # Rotation degrees for data augmentation
        translate=0.1,                  # Translation for data augmentation
        scale=0.5,                      # Scaling factor for data augmentation
        shear=10,                       # Shear for data augmentation
        perspective=0.0,                # Perspective transformation
        flipud=0.5,                     # Vertical flip
        fliplr=0.5,                     # Horizontal flip
        bgr=0.0,                        # BGR color jitter
        mosaic=1.0,                     # Mosaic augmentation
        mixup=0.0,                      # Mixup data augmentation
        copy_paste=0.0,                 # Copy-paste augmentation
        copy_paste_mode="flip",         # Mode for copy-paste augmentation
        auto_augment="randaugment",     # Use Random Augmentation
        erasing=0.4,                    # Erasing data augmentation
        crop_fraction=1.0,              # Crop fraction for augmentation
        save=True,                      # Save model weights after training
        save_period=-1,                 # Save model weights every N epochs (set to -1 to disable)
        val=True,                       # Enable validation during training
        plots=True,                     # Plot metrics during training
        workers=8,                      # Number of workers for data loading
        device=0,                       # Use the first GPU (change if using multiple GPUs)
        verbose=True                    # Enable verbose output during training
    )
