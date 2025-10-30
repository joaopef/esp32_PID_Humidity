
## Controlador de Humidade PID para ESP32



Este repositório contém o código-fonte para um sistema de controlo de qualidade do ar interior, focado na regulação da humidade relativa através de um controlador PID implementado num ESP32.



O desenvolvimento deste projeto foi dividido em duas fases principais, cada uma com o seu próprio código-fonte contido neste repositório.



**Para a análise completa, a documentação e os resultados, visite a página do projeto no meu portfólio:**

**\[➡️ Ver Documentação do Projeto](https://joaopef.github.io/theSTART/pt/Air%20Monitor/)**



---



### Estrutura do Projeto



#### 📂 `01\_pid\_simulation/`



Esta pasta contém o código da fase de desenvolvimento e validação. Este firmware implementa o controlador PID num ambiente de simulação pura, utilizando um modelo matemático para representar o quarto. O objetivo era sintonizar os ganhos do PID e validar a sua performance antes da implementação física. Todos os dados da simulação eram publicados num broker MQTT (HiveMQ) para análise no Grafana.



#### 📂 `02\_gree\_ac\_controller/`



Esta pasta contém o código da \*\*fase de implementação real\*\*. Este firmware utiliza o algoritmo PID validado para controlar um ar condicionado da marca Gree (Inverter) através da rede Wi-Fi local. O objetivo é manter a humidade do quarto num setpoint definido, utilizando uma estratégia de controlo multi-nível para otimizar a eficiência e a vida útil do equipamento.

