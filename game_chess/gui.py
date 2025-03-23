import pygame
import chess

# กำหนดค่าขนาดกระดาน
WIDTH, HEIGHT = 800, 800
SQ_SIZE = WIDTH // 8
WHITE = (240, 217, 181)
BROWN = (181, 136, 99)

# สร้างกระดานหมากรุก
board = chess.Board()

# ฟังก์ชันสำหรับวาดกระดาน
def draw_board(screen):
    for row in range(8):
        for col in range(8):
            color = WHITE if (row + col) % 2 == 0 else BROWN
            pygame.draw.rect(screen, color, (col * SQ_SIZE, row * SQ_SIZE, SQ_SIZE, SQ_SIZE))

# เริ่มต้น pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Chess Game")

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # วาดกระดาน
    draw_board(screen)

    pygame.display.flip()

pygame.quit()
