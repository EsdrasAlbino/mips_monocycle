# Relatório Projeto Final

## Introdução
Este projeto tem como objetivo desenvolver um processador monociclo MIPS utilizando Verilog HDL, com instruçÕes baseados em assembly em uma implementação de C++. O processador será capaz de executar um subconjunto de instruções MIPS e incluirá componentes como uma memória de instruções, memória de dados, ALU e unidade de controle.

![Processador mips completo](https://github.com/user-attachments/assets/450aac2e-646f-4ef6-9c8a-9829f0010b3d)

## Metodologia
O projeto será dividido em várias etapas:
1. Elaborar o código em alto nível, em linguagem C++.
2. Escrever os comandos em assembly, projetar a arquitetura do processador monociclo MIPS, traduzindo para código binário hexadecimal.
3. Implementar os módulos de memória de instruções e memória de dados.
4. Desenvolver a ALU e a unidade de controle.
5. Integrar todos os componentes e testar a funcionalidade do processador.
6. Escrever testbenches para verificar a corretude do projeto.

## Definição dos módulos:

Nesta sessão, trabalharemos cada um dos módulos que constituem a microarquitetura do processador MIPS, especificando sua implementação e lógica de funcionamento:




### Memória de instrução
---
**Função:** Memória de instruções.

- **Entradas:**
    - `address` (32 bits): Endereço de memória.
- **Saída:**
    - `ReadData` (32 bits): Instrução lida da memória.
- **Descrição:** Armazena as instruções a serem executadas e permite a leitura das instruções com base no endereço fornecido.

### Memória de dados
---
**Função:** Memória de dados para operações de load e store.

- **Entradas:**
    - `clock`: Sinal de clock.
    - `address` (32 bits): Endereço de memória.
    - `WriteData` (32 bits): Dados a serem escritos na memória.
    - `WriteEnable`: Sinal de controle para escrita.
- **Saída:**
    - `ReadData` (32 bits): Dados lidos da memória.
- **Descrição:** Permite leitura e escrita de dados na memória. A escrita ocorre na borda de subida do clock.

### Banco de Registros
---
**Função:** Banco de registradores de 32 bits.

- **Entradas:**
    - `clock`: Sinal de clock.
    - `RegWrite`: Sinal que habilita a escrita.
    - `ReadReg1`, `ReadReg2` (5 bits cada): Endereços para leitura.
    - `WriteReg` (5 bits): Endereço para escrita.
    - `WriteData` (32 bits): Dados a serem escritos.
- **Saídas:**
    - `ReadData1`, `ReadData2` (32 bits cada): Dados lidos dos registradores.
- **Descrição:** Realiza leituras assíncronas e escrita síncrona nos registradores. Permite leitura e escrita de valores nos registradores com a escrita controlada pelo clock e um sinal de habilitação.

### ALU
---
**Função:** Unidade Lógica e Aritmética.

- **Entradas:**
    - `ALUCtl` (3 bits): Código de controle da operação.
    - `SrcA`, `SrcB` (32 bits cada): Operandos da ALU.
- **Saídas:**
    - `Zero`: Sinal que indica se o resultado da operação é zero.
    - `ALUOut` (32 bits): Resultado da operação.
- **Descrição:** Realiza operações aritméticas e lógicas como AND, OR, soma, subtração, e menor que (slt), com base no valor de `ALUCtl`.

### ControlUnit
---
**Função:** Unidade de controle principal do processador.

- **Entradas:**
    - `OPCode` (6 bits): Código de operação da instrução.
    - `Funct` (6 bits): Código da função específica da instrução R-type.
- **Saídas:**
    - `MemtoReg`, `MemWrite`, `Branch`, `ALUSrc`, `RegDst`, `RegWrite`, `Jump`: Sinais de controle.
    - `ALUCtl` (3 bits): Código de controle gerado para a ALU.
- **Descrição:** Gera sinais de controle com base no valor do `OPCode` e `Funct`, determinando como as outras partes do processador devem operar.

### Multiplexadores
---
**Função:** Selecionar entre diferentes sinais de entrada com base em sinais de controle.

#### Mux1_REG
- **Entradas:**
    - `inst20_16`, `inst15_11` (5 bits cada): Entradas do multiplexador.
    - `RegDst`: Sinal de controle.
- **Saída:**
    - `WriteReg` (5 bits): Registrador selecionado.
- **Descrição:** Seleciona entre `inst[20:16]` e `inst[15:11]` com base no sinal `RegDst`.

#### Mux2_REG
- **Entradas:**
    - `MemtoReg`: Sinal de controle.
    - `ALUOut`, `ReadData` (32 bits cada): Entradas do multiplexador.
- **Saída:**
    - `WriteData` (32 bits): Dados selecionados para escrita.
- **Descrição:** Seleciona entre `ALUOut` e `ReadData` com base no sinal `MemtoReg`.

#### Mux3_ALU
- **Entradas:**
    - `ALUSrc`: Sinal de controle.
    - `ReadData2`, `Extend32` (32 bits cada): Entradas do multiplexador.
- **Saída:**
    - `SrcB` (32 bits): Entrada B selecionada.
- **Descrição:** Seleciona entre `ReadData2` e `Extend32` com base no sinal `ALUSrc`.

#### Mux4_Src
- **Entradas:**
    - `PCSrc`: Sinal de controle.
    - `PCPlus4`, `PCBranch` (32 bits cada): Entradas do multiplexador.
- **Saída:**
    - `PCin` (32 bits): Próximo valor do PC.
- **Descrição:** Seleciona entre `PCPlus4` e `PCBranch` com base no sinal `PCSrc`.

#### Mux5_PC
- **Entradas:**
    - `pcPlus4`, `Branch` (32 bits cada): Entradas do multiplexador.
    - `PCSrc`: Sinal de controle.
- **Saída:**
    - `PCin` (32 bits): Próximo valor do PC.
- **Descrição:** Seleciona entre `pcPlus4` e `Branch` com base no sinal `PCSrc`.

#### Mux6_Jump
- **Entradas:**
    - `PCJumpIn` (32 bits): Entrada do PC para salto.
    - `Jump`: Sinal de controle.
    - `Destination` (26 bits): Destino do salto.
- **Saída:**
    - `PCJumpOut` (32 bits): Valor de saída para o salto.
- **Descrição:** Seleciona o endereço de salto concatenando `Destination` com os bits mais significativos de `PCJumpIn` se `Jump` estiver ativo.

### PC
---
**Função:** Contador de Programa.

- **Entradas:**
    - `clock`: Sinal de clock.
    - `PCin` (32 bits): Próximo valor do PC.
- **Saída:**
    - `PCout` (32 bits): Valor atual do PC.
- **Descrição:** Armazena o valor do PC e atualiza-o na borda de subida do clock.

### SignExtend
---
**Função:** Extensor de sinal.

- **Entradas:**
    - `inst15_0` (16 bits): Valor de entrada a ser estendido.
- **Saída:**
    - `Extend32` (32 bits): Valor de saída estendido.
- **Descrição:** Realiza a extensão de sinal do valor de entrada de 16 bits para 32 bits.

### PCBranch
---
**Função:** Calcula o endereço de ramificação.

- **Entradas:**
    - `Extend32` (32 bits): Valor de deslocamento estendido.
    - `PCplus4` (32 bits): Valor do PC incrementado.
- **Saída:**
    - `out` (32 bits): Endereço de ramificação.
- **Descrição:** Soma `Extend32` deslocado à esquerda por 2 com `PCplus4` para calcular o endereço de ramificação.

### PCplus_4
---
**Função:** Calcula o próximo valor do PC.

- **Entradas:**
    - `PCplusIn` (32 bits): Valor atual do PC.
- **Saída:**
    - `PCplusOut` (32 bits): Próximo valor do PC.
- **Descrição:** Adiciona 4 ao valor atual do PC para calcular o próximo valor do PC.
## Resultados
O processador monociclo MIPS foi implementado e testado com sucesso. Ele foi capaz de executar uma variedade de instruções MIPS e produziu os resultados esperados. O projeto atendeu aos requisitos de desempenho e demonstrou o comportamento correto.

## Conclusão
Em conclusão, este projeto proporcionou uma valiosa experiência prática no projeto e implementação de um processador monociclo MIPS utilizando Verilog HDL. O projeto ajudou a aprofundar o entendimento da arquitetura de computadores e do projeto lógico digital. Trabalhos futuros podem envolver a expansão do conjunto de instruções ou a otimização do projeto para melhor desempenho.
















