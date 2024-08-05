.data
    v0: .word 0     # Reservar espaço para v0
    v1: .word 0     # Reservar espaço para v1

.text
    main:
        # Inicializar v0 com 5
        li $t0, 5          # Carregar o valor 5 em $t0
        sw $t0, v0         # Armazenar $t0 (5) em v0

        # Inicializar v1 com 10
        li $t1, 10         # Carregar o valor 10 em $t1
        sw $t1, v1         # Armazenar $t1 (10) em v1

        # Carregar valores de v0 e v1 em registradores
        lw $t0, v0         # $t0 = v0
        lw $t1, v1         # $t1 = v1

        # Comparar v1 com v0
        beq $t0, $t1, equal # Se v0 == v1, saltar para 'equal'

        # Código quando v1 != v0
        j end              # Saltar para 'end'

    equal:
        # v0 = 1
        li $t2, 1          # Carregar o valor 1 em $t2
        sw $t2, v0         # Armazenar $t2 (1) em v0

    end:
        # Finalizar o programa
        li $v0, 10         # Código para sair do programa
        syscall
