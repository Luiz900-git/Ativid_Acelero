import pandas as pd
import matplotlib.pyplot as plt

# Após o upload, verifique o nome do arquivo
filename = 'mpu_data.csv'  # ajuste se necessário

# Carregar o CSV
df = pd.read_csv(filename)

# Verifique as primeiras linhas para garantir que o arquivo foi carregado corretamente
print(df.head())

# Plot acelerômetro
plt.figure(figsize=(10, 6))
plt.plot(df['AccelX'], label='Accel X')
plt.plot(df['AccelY'], label='Accel Y')
plt.plot(df['AccelZ'], label='Accel Z')
plt.title('Acelerômetro - Coordenadas X, Y e Z')
plt.xlabel('Amostras')
plt.ylabel('Aceleração (g)')
plt.legend()
plt.grid(True)
plt.show()

# Plot giroscópio
plt.figure(figsize=(10, 6))
plt.plot(df['GyroX'], label='Gyro X')
plt.plot(df['GyroY'], label='Gyro Y')
plt.plot(df['GyroZ'], label='Gyro Z')
plt.title('Giroscópio - Coordenadas X, Y e Z')
plt.xlabel('Amostras')
plt.ylabel('Velocidade Angular (°/s)')
plt.legend()
plt.grid(True)
plt.show()
