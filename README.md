﻿# Atividade com sd e acelerômetro
# Exibição de dados com sensor e sd

**Autor:** [Luiz Eduardo]  

Este projeto utiliza o microcontrolador **Raspberry Pi Pico (RP2040)** em conjunto com um sensor acelerômetro e um micro sd para coleta e visualização de dados. Ainda exibe esses dados com auxílio de um arquivo .py.

## Funcionalidades  

- Leitura do acelerômetro e giroscópio com sensor **[MPU6050]**  
- Simples Menu com os comandos através do monitor serial  
- Exibição de gráficos com código .py 
- Exibição em tempo real no display **OLED SSD1306 (128x64)**  
- Comunicação via barramento **I2C**  
    

## Hardware Necessário  

- **Raspberry Pi Pico (RP2040)**  
- Sensor **[MPU5060]** – I2C  
- Micro SD (2GB mínimo)  
- Display **OLED SSD1306 (128x64)** – I2C   
- Cabos de conexão  
- Leds e Buzzers

## Como Funciona  

O código inicializa o sensore e o sd conectados ao Raspberry Pi Pico por barramentos **I2C**. A leitura dos dados ambientais é feita de acordo com a vontade do usuário, e os resultados são:  

- Exibidos no display OLED e podem ser plotados em gráficos .py 
- Impressos no terminal serial para fins de monitoramento 
  

## Observações  

- Certifique-se de que o sensor esteja conectados corretamente ao barramento **I2C**.  
- Necessário usar um micro sd  
- Necessário ter o Python e as bibliotecas pandas e matplotlib instaladas  
- Monte o sd antes de gravas os dados
- Desmonte o sd antes de desligar o código
- CUIDADO AO FORMATAR O CARTÃO SD  

---  
