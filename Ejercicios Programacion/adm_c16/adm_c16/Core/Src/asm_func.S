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
.global asm_filtroVentana10
.global asm_pack32to16
.global asm_max
.global asm_downsampleM
.global asm_invertir

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
@   void void asm_productoEscalar16 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar);
@
@ Parametros recibidos:
@   r0: Puntero a vector de entrada
@   r1: Puntero a vector de salida
@   r2: longitud de vector
@   r3: escalar

.thumb_func
    asm_productoEscalar16:
        _asm_productoEscalar16_loop:
        	ldrh r4, [r0], 2 //r4 = *vectorIn++
        	mul r4, r4, r3 //r4 = r4 * escalar
        	str r4, [r1], 2 //*vectorOut++ = r4
        	subs r2, 1 //longitud--
        	bne _asm_productoEscalar16_loop //if (longitud != 0) jump to _asm_productoEscalar16_loop
        bx lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void asm_productoEscalar12 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar);

@
@ Parametros recibidos:
@   r0: Puntero a vector de entrada
@   r1: Puntero a vector de salida
@   r2: longitud de vector
@   r3: escalar

.thumb_func
    asm_productoEscalar12:
    	push {r4, r5}
    	mov r5, 0xfff
        _asm_productoEscalar12_loop:
        	ldrh r4, [r0], 2 //r4 = *vectorIn++
        	mul r4, r4, r3 //r4 = r4 * escalar
        	cmp r4, r5
        	bcc _asm_productoEscalar12_noOverflow
        	mov r4, r5
        	_asm_productoEscalar12_noOverflow:
        	strh r4, [r1], 2 //*vectorOut++ = r4
        	subs r2, 1 //longitud--
        	bne _asm_productoEscalar12_loop //if (longitud != 0) jump to _asm_productoEscalar12_loop
        pop {r4, r5}
        bx lr


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@ void asm_filtroVentana10(uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitudVectorIn);

@
@ Parametros recibidos:
@   r0: Puntero a vector de entrada
@   r1: Puntero a vector de salida
@   r2: longitud de vector

.thumb_func
    asm_filtroVentana10:
    	push {r4, r5, r6, r7, r8}
    	mov r4, 0 //r4 se usa para almacenar la suma
    	mov r5, 0 //r5 la cuenta del primer loop i
    	mov r6, 0 //r6 guarda el siguiente acceso al vector
    	mov r7, 0 //r7 guarda la cuenta del segundo loop j
    	mov r8, r2 //guarda el valor inicial de longitud
    	add r8, r8
        _asm_filtroVentana10_loop:
        	mov r6, r5 //r6 = r5
        	mov r7, 10 //j = 10
        	_asm_filtroVentana10_loop_suma:
				cmp r6, r8 //hace overflow en caso de que el valor de acceso al vector es mayor a su longitud
        		bne _asm_filtroVentana10_no_overflow
        		sub r6, r8
        		_asm_filtroVentana10_no_overflow:
        		ldrh r3, [r0, r6] //r3 = *(vectorIn + r6)
        		add r4, r4, r3 //suma += r3
        		add r6, 2 //r6 += 2
        		subs r7, 1 //j++
        		bne _asm_filtroVentana10_loop_suma //if(r7 != 0 goto _asm_filtroVentana10_loop_suma)
        	mov r7, 10
        	udiv r4, r4, r7 //r4 = r4 / 10
        	strh r4, [r1], 2 //*vectorOut++ = r4
        	mov r4, 0 //r4 = 0
        	add r5, 2 //i++
        	subs r2, 1 //longitud--
        	bne _asm_filtroVentana10_loop //if (longitud != 0) jump to _asm_filtroVentana10_loop
        pop {r4, r5, r6, r7, r8}
        bx lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@ void asm_pack32to16(int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud);

@
@ Parametros recibidos:
@   r0: Puntero a vector de entrada
@   r1: Puntero a vector de salida
@   r2: longitud de vector

.thumb_func
    asm_pack32to16:
        _asm_pack32to16_loop:
        	ldr r3, [r0], 4 //r3 = *vectorIn++
        	lsr r3, 16
        	strh r3, [r1], 2 //*vectorOut++ = r4
        	subs r2, 1 //longitud--
        	bne _asm_pack32to16_loop //if (longitud != 0) jump to _asm_pack32to16_loop
        bx lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@ int32_t asm_max(int32_t * vectorIn, uint32_t longitud);
@
@ Parametros recibidos:
@   r0: Puntero a vector de entrada
@   r1: longitud de vector

.thumb_func
    asm_max:
    	ldr r2, [r0] //r2 se usa para guardar el valor maximo
        _asm_max_loop:
        	ldr r3, [r0], 4 //r3 = *vectorIn++
        	cmp r3, r2
        	blt _asm_max_not_greater
        	mov r2, r3 //r2 = r3
        	_asm_max_not_greater:
        	subs r1, 1 //longitud--
        	bne _asm_max_loop //if (longitud != 0) jump to _asm_max_loop
        mov r0, r2
        bx lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@ void asm_downsampleM(int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N);
@
@ Parametros recibidos:
@   r0: Puntero a vector de entrada
@   r1: puntero a vector de salida
@	r2: Longitud del vector
@	r3: Salto del downsampling

.thumb_func
    asm_downsampleM:
    	push {r4, r5}
    	mov r4, 0 //r4 se usa para guardar el valor del contador
        _asm_downsampleM_loop:
        	cmp r4, r3
        	bcc _asm_downsampleM_counter_ok
        		mov r4, 0 //counter = 0
        		add r0, 4 //vectorIn++
        		b _asm_downsampleM_loop
        	_asm_downsampleM_counter_ok:
        	ldr r5, [r0], 4 //r5 = *vectorIn++
        	str r5, [r1], 4 //*vectorOut++ = r5
        	add r4, 1 //counter++
        	subs r2, 1 //longitud--
        	bne _asm_downsampleM_loop //if (longitud != 0) jump to _asm_downsampleM_loop
        pop {r4, r5}
        bx lr


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@ void asm_invertir(uint16_t * vector, uint32_t longitud);
@
@ Parametros recibidos:
@   r0: Puntero a vector
@   r1: Longitud del vector

.thumb_func
    asm_invertir:
    	push {r4}
    	mov r2, r0 //r2 se usa para guardar de forma auxiliar el contenido del vector
    	add r2, r1
    	add r2, r1
    	sub r2, 2
    	mov r3, 0 //r3 se usa para guardar de forma auxiliar el contenido del vector
    	mov r4, 0 //r4 se usa para guardar de forma auxiliar el contenido del vector
        _asm_invertir_loop:
			ldrh r3, [r0] //r3 = *vector
			ldrh r4, [r2] //r4 = *(vector + longitud)
			strh r4, [r0], 2//*vector = r4
			strh r3, [r2], -2 //*(vector + longitud) = r3
        	subs r1, 2 //i++
        	bne _asm_invertir_loop //if (longitud != 0) jump to asm_invertir
        pop {r4}
        bx lr



