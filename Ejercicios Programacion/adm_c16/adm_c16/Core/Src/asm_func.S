.syntax unified
.thumb
.text

@ Simbolos exportados
.global asm_svc
.global asm_sum
.global asm_zeros
.global asm_productoEscalar32
.global asm_productoEscalar16
.global asm_productoEscalar12

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void asm_svc (void)
@
.thumb_func
    asm_svc:
        svc 0
        bx lr


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   uint32_t asmSum (uint32_t firstOperand, uint32_t secondOperand)
@
@ Parametros recibidos:
@   r0: firstOperand
@   r1: secondOperand
@
@ Valor de retorno:
@   r0: resultado de la suma de firstOperand y secondOperand
@
.thumb_func
    asm_sum:
        add r0, r1  @ r0 = r0 + r1
        bx lr       @ vuelve adonde fue llamada (especificamente, si "pc"
                    @ es la direccion, en el codigo del programa, de la
                    @ instruccion "bl asm_sum" usada para saltar a
                    @ esta funcion, entonces "lr" contiene ese "pc" + 4).
                    @ el valor de retorno debe quedar en el registro r0.
                    @ En este caso, r0 es el resultado de r0 + r1.

        @ Otras alternativas para el retorno
        @ mov pc,lr
        @ bx lr
        @ push {lr} ... pop {pc}


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void asm_zeros (uint32_t * vector, uint32_t longitud)
@
@ Parametros recibidos:
@   r0: Puntero a vector
@   r1: longitud de vector

.thumb_func
    asm_zeros:
        mov r2, 0
        _asm_zeros_loop:
        	str r2, [r0], 4 //vector++ = 0;
        	subs r1, 1 //longitud--
        	bne _asm_zeros_loop //if (longitud != 0) jump to _asm_zeros_loop
        bx lr


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void asm_productoEscalar32(uint32_t* vectorIn, uint32_t * vectorOut, uint32_t longitud, uint32_t escalar);
@
@ Parametros recibidos:
@   r0: Puntero a vector de entrada
@   r1: Puntero a vector de salida
@   r2: longitud de vector
@   r3: escalar

.thumb_func
    asm_productoEscalar32:
        _asm_productoEscalar32_loop:
        	ldr r4, [r0], 4 //r4 = *vectorIn++
        	mul r4, r4, r3 //r4 = r4 * escalar
        	str r4, [r1], 4 //*vectorOut++ = r4
        	subs r2, 1 //longitud--
        	bne _asm_productoEscalar32_loop //if (longitud != 0) jump to _asm_productoEscalar32_loop
        bx lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void void asm_productoEscalar16 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar);(uint32_t* vectorIn, uint32_t * vectorOut, uint32_t longitud, uint32_t escalar);
@
@ Parametros recibidos:
@   r0: Puntero a vector de entrada
@   r1: Puntero a vector de salida
@   r2: longitud de vector
@   r3: escalar

.thumb_func
    asm_productoEscalar16:
        _asm_productoEscalar16_loop:
        	ldr r4, [r0], 2 //r4 = *vectorIn++
        	mul r4, r4, r3 //r4 = r4 * escalar
        	str r4, [r1], 2 //*vectorOut++ = r4
        	subs r2, 1 //longitud--
        	bne _asm_productoEscalar16_loop //if (longitud != 0) jump to _asm_productoEscalar16_loop
        bx lr


