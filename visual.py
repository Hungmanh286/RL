import pygame

def draw_env_pygame(screen, env, cell_size=40):
    screen.fill((255, 255, 255))  # clear background

    # Vẽ lưới
    for row in range(env.n_rows):
        for col in range(env.n_cols):
            rect = pygame.Rect(col * cell_size, row * cell_size, cell_size, cell_size)

            if env.grid[row][col] == 1:  # vật cản
                pygame.draw.rect(screen, (0, 0, 0), rect)  # fill đen
            else:
                pygame.draw.rect(screen, (220, 220, 220), rect, 1)  # ô trống

    # Vẽ packages
    for pkg in env.packages:
        if pkg.status == 'waiting':
            row, col = pkg.start
            color = (255, 165, 0)  # cam
        elif pkg.status == 'in_transit':
            row, col = pkg.target
            color = (0, 255, 0)  # xanh lá
        else:
            continue

        x = col * cell_size + cell_size // 2
        y = row * cell_size + cell_size // 2

        pygame.draw.circle(screen, color, (x, y), cell_size // 4)

    # Vẽ robots
    for robot in env.robots:
        row, col = robot.position
        x = col * cell_size + cell_size // 2
        y = row * cell_size + cell_size // 2

        pygame.draw.circle(screen, (0, 0, 255), (x, y), cell_size // 3)
        if robot.carrying > 0:
            pygame.draw.circle(screen, (255, 215, 0), (x, y), cell_size // 3, 2)

    pygame.display.flip()
