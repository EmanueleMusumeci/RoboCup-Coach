(define (problem simplestriker)
(:domain robocupdeterministic)
(:objects robot ball - movable 
    wpc0r0 wpc0r1 wpc0r2 wpc0r3 wpc0r4 wpc0r5 wpc0r6 wpc0r7 wpc0r8 wpc0r9 wpc0r10 wpc0r11 wpc0r12 wpc0r13 wpc0r14 wpc0r15 wpc1r0 wpc1r1 wpc1r2 wpc1r3 wpc1r4 wpc1r5 wpc1r6 wpc1r7 wpc1r8 wpc1r9 wpc1r10 wpc1r11 wpc1r12 wpc1r13 wpc1r14 wpc1r15 wpc2r0 wpc2r1 wpc2r2 wpc2r3 wpc2r4 wpc2r5 wpc2r6 wpc2r7 wpc2r8 wpc2r9 wpc2r10 wpc2r11 wpc2r12 wpc2r13 wpc2r14 wpc2r15 wpc3r0 wpc3r1 wpc3r2 wpc3r3 wpc3r4 wpc3r5 wpc3r6 wpc3r7 wpc3r8 wpc3r9 wpc3r10 wpc3r11 wpc3r12 wpc3r13 wpc3r14 wpc3r15 wpc4r0 wpc4r1 wpc4r2 wpc4r3 wpc4r4 wpc4r5 wpc4r6 wpc4r7 wpc4r8 wpc4r9 wpc4r10 wpc4r11 wpc4r12 wpc4r13 wpc4r14 wpc4r15 wpc5r0 wpc5r1 wpc5r2 wpc5r3 wpc5r4 wpc5r5 wpc5r6 wpc5r7 wpc5r8 wpc5r9 wpc5r10 wpc5r11 wpc5r12 wpc5r13 wpc5r14 wpc5r15 wpc6r0 wpc6r1 wpc6r2 wpc6r3 wpc6r4 wpc6r5 wpc6r6 wpc6r7 wpc6r8 wpc6r9 wpc6r10 wpc6r11 wpc6r12 wpc6r13 wpc6r14 wpc6r15 wpc7r0 wpc7r1 wpc7r2 wpc7r3 wpc7r4 wpc7r5 wpc7r6 wpc7r7 wpc7r8 wpc7r9 wpc7r10 wpc7r11 wpc7r12 wpc7r13 wpc7r14 wpc7r15 wpc8r0 wpc8r1 wpc8r2 wpc8r3 wpc8r4 wpc8r5 wpc8r6 wpc8r7 wpc8r8 wpc8r9 wpc8r10 wpc8r11 wpc8r12 wpc8r13 wpc8r14 wpc8r15 wpc9r0 wpc9r1 wpc9r2 wpc9r3 wpc9r4 wpc9r5 wpc9r6 wpc9r7 wpc9r8 wpc9r9 wpc9r10 wpc9r11 wpc9r12 wpc9r13 wpc9r14 wpc9r15 wpc10r0 wpc10r1 wpc10r2 wpc10r3 wpc10r4 wpc10r5 wpc10r6 wpc10r7 wpc10r8 wpc10r9 wpc10r10 wpc10r11 wpc10r12 wpc10r13 wpc10r14 wpc10r15 wpc11r0 wpc11r1 wpc11r2 wpc11r3 wpc11r4 wpc11r5 wpc11r6 wpc11r7 wpc11r8 wpc11r9 wpc11r10 wpc11r11 wpc11r12 wpc11r13 wpc11r14 wpc11r15 wpc12r0 wpc12r1 wpc12r2 wpc12r3 wpc12r4 wpc12r5 wpc12r6 wpc12r7 wpc12r8 wpc12r9 wpc12r10 wpc12r11 wpc12r12 wpc12r13 wpc12r14 wpc12r15 wpc13r0 wpc13r1 wpc13r2 wpc13r3 wpc13r4 wpc13r5 wpc13r6 wpc13r7 wpc13r8 wpc13r9 wpc13r10 wpc13r11 wpc13r12 wpc13r13 wpc13r14 wpc13r15 wpc14r0 wpc14r1 wpc14r2 wpc14r3 wpc14r4 wpc14r5 wpc14r6 wpc14r7 wpc14r8 wpc14r9 wpc14r10 wpc14r11 wpc14r12 wpc14r13 wpc14r14 wpc14r15 wpc15r0 wpc15r1 wpc15r2 wpc15r3 wpc15r4 wpc15r5 wpc15r6 wpc15r7 wpc15r8 wpc15r9 wpc15r10 wpc15r11 wpc15r12 wpc15r13 wpc15r14 wpc15r15 - location
    )
(:init 
    (isrobot robot) (isball ball) 
(adjacent wpc0r1 wpc0r0)
(adjacent wpc0r2 wpc0r1)
(adjacent wpc0r3 wpc0r2)
(adjacent wpc0r4 wpc0r3)
(adjacent wpc0r5 wpc0r4)
(adjacent wpc0r6 wpc0r5)
(adjacent wpc0r7 wpc0r6)
(adjacent wpc0r8 wpc0r7)
(adjacent wpc0r9 wpc0r8)
(adjacent wpc0r10 wpc0r9)
(adjacent wpc0r11 wpc0r10)
(adjacent wpc0r12 wpc0r11)
(adjacent wpc0r13 wpc0r12)
(adjacent wpc0r14 wpc0r13)
(adjacent wpc0r15 wpc0r14)
(adjacent wpc1r0 wpc0r0)
(adjacent wpc1r1 wpc0r1)
(adjacent wpc1r1 wpc1r0)
(adjacent wpc1r2 wpc0r2)
(adjacent wpc1r2 wpc1r1)
(adjacent wpc1r3 wpc0r3)
(adjacent wpc1r3 wpc1r2)
(adjacent wpc1r4 wpc0r4)
(adjacent wpc1r4 wpc1r3)
(adjacent wpc1r5 wpc0r5)
(adjacent wpc1r5 wpc1r4)
(adjacent wpc1r6 wpc0r6)
(adjacent wpc1r6 wpc1r5)
(adjacent wpc1r7 wpc0r7)
(adjacent wpc1r7 wpc1r6)
(adjacent wpc1r8 wpc0r8)
(adjacent wpc1r8 wpc1r7)
(adjacent wpc1r9 wpc0r9)
(adjacent wpc1r9 wpc1r8)
(adjacent wpc1r10 wpc0r10)
(adjacent wpc1r10 wpc1r9)
(adjacent wpc1r11 wpc0r11)
(adjacent wpc1r11 wpc1r10)
(adjacent wpc1r12 wpc0r12)
(adjacent wpc1r12 wpc1r11)
(adjacent wpc1r13 wpc0r13)
(adjacent wpc1r13 wpc1r12)
(adjacent wpc1r14 wpc0r14)
(adjacent wpc1r14 wpc1r13)
(adjacent wpc1r15 wpc0r15)
(adjacent wpc1r15 wpc1r14)
(adjacent wpc2r0 wpc1r0)
(adjacent wpc2r1 wpc1r1)
(adjacent wpc2r1 wpc2r0)
(adjacent wpc2r2 wpc1r2)
(adjacent wpc2r2 wpc2r1)
(adjacent wpc2r3 wpc1r3)
(adjacent wpc2r3 wpc2r2)
(adjacent wpc2r4 wpc1r4)
(adjacent wpc2r4 wpc2r3)
(adjacent wpc2r5 wpc1r5)
(adjacent wpc2r5 wpc2r4)
(adjacent wpc2r6 wpc1r6)
(adjacent wpc2r6 wpc2r5)
(adjacent wpc2r7 wpc1r7)
(adjacent wpc2r7 wpc2r6)
(adjacent wpc2r8 wpc1r8)
(adjacent wpc2r8 wpc2r7)
(adjacent wpc2r9 wpc1r9)
(adjacent wpc2r9 wpc2r8)
(adjacent wpc2r10 wpc1r10)
(adjacent wpc2r10 wpc2r9)
(adjacent wpc2r11 wpc1r11)
(adjacent wpc2r11 wpc2r10)
(adjacent wpc2r12 wpc1r12)
(adjacent wpc2r12 wpc2r11)
(adjacent wpc2r13 wpc1r13)
(adjacent wpc2r13 wpc2r12)
(adjacent wpc2r14 wpc1r14)
(adjacent wpc2r14 wpc2r13)
(adjacent wpc2r15 wpc1r15)
(adjacent wpc2r15 wpc2r14)
(adjacent wpc3r0 wpc2r0)
(adjacent wpc3r1 wpc2r1)
(adjacent wpc3r1 wpc3r0)
(adjacent wpc3r2 wpc2r2)
(adjacent wpc3r2 wpc3r1)
(adjacent wpc3r3 wpc2r3)
(adjacent wpc3r3 wpc3r2)
(adjacent wpc3r4 wpc2r4)
(adjacent wpc3r4 wpc3r3)
(adjacent wpc3r5 wpc2r5)
(adjacent wpc3r5 wpc3r4)
(adjacent wpc3r6 wpc2r6)
(adjacent wpc3r6 wpc3r5)
(adjacent wpc3r7 wpc2r7)
(adjacent wpc3r7 wpc3r6)
(adjacent wpc3r8 wpc2r8)
(adjacent wpc3r8 wpc3r7)
(adjacent wpc3r9 wpc2r9)
(adjacent wpc3r9 wpc3r8)
(adjacent wpc3r10 wpc2r10)
(adjacent wpc3r10 wpc3r9)
(adjacent wpc3r11 wpc2r11)
(adjacent wpc3r11 wpc3r10)
(adjacent wpc3r12 wpc2r12)
(adjacent wpc3r12 wpc3r11)
(adjacent wpc3r13 wpc2r13)
(adjacent wpc3r13 wpc3r12)
(adjacent wpc3r14 wpc2r14)
(adjacent wpc3r14 wpc3r13)
(adjacent wpc3r15 wpc2r15)
(adjacent wpc3r15 wpc3r14)
(adjacent wpc4r0 wpc3r0)
(adjacent wpc4r1 wpc3r1)
(adjacent wpc4r1 wpc4r0)
(adjacent wpc4r2 wpc3r2)
(adjacent wpc4r2 wpc4r1)
(adjacent wpc4r3 wpc3r3)
(adjacent wpc4r3 wpc4r2)
(adjacent wpc4r4 wpc3r4)
(adjacent wpc4r4 wpc4r3)
(adjacent wpc4r5 wpc3r5)
(adjacent wpc4r5 wpc4r4)
(adjacent wpc4r6 wpc3r6)
(adjacent wpc4r6 wpc4r5)
(adjacent wpc4r7 wpc3r7)
(adjacent wpc4r7 wpc4r6)
(adjacent wpc4r8 wpc3r8)
(adjacent wpc4r8 wpc4r7)
(adjacent wpc4r9 wpc3r9)
(adjacent wpc4r9 wpc4r8)
(adjacent wpc4r10 wpc3r10)
(adjacent wpc4r10 wpc4r9)
(adjacent wpc4r11 wpc3r11)
(adjacent wpc4r11 wpc4r10)
(adjacent wpc4r12 wpc3r12)
(adjacent wpc4r12 wpc4r11)
(adjacent wpc4r13 wpc3r13)
(adjacent wpc4r13 wpc4r12)
(adjacent wpc4r14 wpc3r14)
(adjacent wpc4r14 wpc4r13)
(adjacent wpc4r15 wpc3r15)
(adjacent wpc4r15 wpc4r14)
(adjacent wpc5r0 wpc4r0)
(adjacent wpc5r1 wpc4r1)
(adjacent wpc5r1 wpc5r0)
(adjacent wpc5r2 wpc4r2)
(adjacent wpc5r2 wpc5r1)
(adjacent wpc5r3 wpc4r3)
(adjacent wpc5r3 wpc5r2)
(adjacent wpc5r4 wpc4r4)
(adjacent wpc5r4 wpc5r3)
(adjacent wpc5r5 wpc4r5)
(adjacent wpc5r5 wpc5r4)
(adjacent wpc5r6 wpc4r6)
(adjacent wpc5r6 wpc5r5)
(adjacent wpc5r7 wpc4r7)
(adjacent wpc5r7 wpc5r6)
(adjacent wpc5r8 wpc4r8)
(adjacent wpc5r8 wpc5r7)
(adjacent wpc5r9 wpc4r9)
(adjacent wpc5r9 wpc5r8)
(adjacent wpc5r10 wpc4r10)
(adjacent wpc5r10 wpc5r9)
(adjacent wpc5r11 wpc4r11)
(adjacent wpc5r11 wpc5r10)
(adjacent wpc5r12 wpc4r12)
(adjacent wpc5r12 wpc5r11)
(adjacent wpc5r13 wpc4r13)
(adjacent wpc5r13 wpc5r12)
(adjacent wpc5r14 wpc4r14)
(adjacent wpc5r14 wpc5r13)
(adjacent wpc5r15 wpc4r15)
(adjacent wpc5r15 wpc5r14)
(adjacent wpc6r0 wpc5r0)
(adjacent wpc6r1 wpc5r1)
(adjacent wpc6r1 wpc6r0)
(adjacent wpc6r2 wpc5r2)
(adjacent wpc6r2 wpc6r1)
(adjacent wpc6r3 wpc5r3)
(adjacent wpc6r3 wpc6r2)
(adjacent wpc6r4 wpc5r4)
(adjacent wpc6r4 wpc6r3)
(adjacent wpc6r5 wpc5r5)
(adjacent wpc6r5 wpc6r4)
(adjacent wpc6r6 wpc5r6)
(adjacent wpc6r6 wpc6r5)
(adjacent wpc6r7 wpc5r7)
(adjacent wpc6r7 wpc6r6)
(adjacent wpc6r8 wpc5r8)
(adjacent wpc6r8 wpc6r7)
(adjacent wpc6r9 wpc5r9)
(adjacent wpc6r9 wpc6r8)
(adjacent wpc6r10 wpc5r10)
(adjacent wpc6r10 wpc6r9)
(adjacent wpc6r11 wpc5r11)
(adjacent wpc6r11 wpc6r10)
(adjacent wpc6r12 wpc5r12)
(adjacent wpc6r12 wpc6r11)
(adjacent wpc6r13 wpc5r13)
(adjacent wpc6r13 wpc6r12)
(adjacent wpc6r14 wpc5r14)
(adjacent wpc6r14 wpc6r13)
(adjacent wpc6r15 wpc5r15)
(adjacent wpc6r15 wpc6r14)
(adjacent wpc7r0 wpc6r0)
(adjacent wpc7r1 wpc6r1)
(adjacent wpc7r1 wpc7r0)
(adjacent wpc7r2 wpc6r2)
(adjacent wpc7r2 wpc7r1)
(adjacent wpc7r3 wpc6r3)
(adjacent wpc7r3 wpc7r2)
(adjacent wpc7r4 wpc6r4)
(adjacent wpc7r4 wpc7r3)
(adjacent wpc7r5 wpc6r5)
(adjacent wpc7r5 wpc7r4)
(adjacent wpc7r6 wpc6r6)
(adjacent wpc7r6 wpc7r5)
(adjacent wpc7r7 wpc6r7)
(adjacent wpc7r7 wpc7r6)
(adjacent wpc7r8 wpc6r8)
(adjacent wpc7r8 wpc7r7)
(adjacent wpc7r9 wpc6r9)
(adjacent wpc7r9 wpc7r8)
(adjacent wpc7r10 wpc6r10)
(adjacent wpc7r10 wpc7r9)
(adjacent wpc7r11 wpc6r11)
(adjacent wpc7r11 wpc7r10)
(adjacent wpc7r12 wpc6r12)
(adjacent wpc7r12 wpc7r11)
(adjacent wpc7r13 wpc6r13)
(adjacent wpc7r13 wpc7r12)
(adjacent wpc7r14 wpc6r14)
(adjacent wpc7r14 wpc7r13)
(adjacent wpc7r15 wpc6r15)
(adjacent wpc7r15 wpc7r14)
(adjacent wpc8r0 wpc7r0)
(adjacent wpc8r1 wpc7r1)
(adjacent wpc8r1 wpc8r0)
(adjacent wpc8r2 wpc7r2)
(adjacent wpc8r2 wpc8r1)
(adjacent wpc8r3 wpc7r3)
(adjacent wpc8r3 wpc8r2)
(adjacent wpc8r4 wpc7r4)
(adjacent wpc8r4 wpc8r3)
(adjacent wpc8r5 wpc7r5)
(adjacent wpc8r5 wpc8r4)
(adjacent wpc8r6 wpc7r6)
(adjacent wpc8r6 wpc8r5)
(adjacent wpc8r7 wpc7r7)
(adjacent wpc8r7 wpc8r6)
(adjacent wpc8r8 wpc7r8)
(adjacent wpc8r8 wpc8r7)
(adjacent wpc8r9 wpc7r9)
(adjacent wpc8r9 wpc8r8)
(adjacent wpc8r10 wpc7r10)
(adjacent wpc8r10 wpc8r9)
(adjacent wpc8r11 wpc7r11)
(adjacent wpc8r11 wpc8r10)
(adjacent wpc8r12 wpc7r12)
(adjacent wpc8r12 wpc8r11)
(adjacent wpc8r13 wpc7r13)
(adjacent wpc8r13 wpc8r12)
(adjacent wpc8r14 wpc7r14)
(adjacent wpc8r14 wpc8r13)
(adjacent wpc8r15 wpc7r15)
(adjacent wpc8r15 wpc8r14)
(adjacent wpc9r0 wpc8r0)
(adjacent wpc9r1 wpc8r1)
(adjacent wpc9r1 wpc9r0)
(adjacent wpc9r2 wpc8r2)
(adjacent wpc9r2 wpc9r1)
(adjacent wpc9r3 wpc8r3)
(adjacent wpc9r3 wpc9r2)
(adjacent wpc9r4 wpc8r4)
(adjacent wpc9r4 wpc9r3)
(adjacent wpc9r5 wpc8r5)
(adjacent wpc9r5 wpc9r4)
(adjacent wpc9r6 wpc8r6)
(adjacent wpc9r6 wpc9r5)
(adjacent wpc9r7 wpc8r7)
(adjacent wpc9r7 wpc9r6)
(adjacent wpc9r8 wpc8r8)
(adjacent wpc9r8 wpc9r7)
(adjacent wpc9r9 wpc8r9)
(adjacent wpc9r9 wpc9r8)
(adjacent wpc9r10 wpc8r10)
(adjacent wpc9r10 wpc9r9)
(adjacent wpc9r11 wpc8r11)
(adjacent wpc9r11 wpc9r10)
(adjacent wpc9r12 wpc8r12)
(adjacent wpc9r12 wpc9r11)
(adjacent wpc9r13 wpc8r13)
(adjacent wpc9r13 wpc9r12)
(adjacent wpc9r14 wpc8r14)
(adjacent wpc9r14 wpc9r13)
(adjacent wpc9r15 wpc8r15)
(adjacent wpc9r15 wpc9r14)
(adjacent wpc10r0 wpc9r0)
(adjacent wpc10r1 wpc9r1)
(adjacent wpc10r1 wpc10r0)
(adjacent wpc10r2 wpc9r2)
(adjacent wpc10r2 wpc10r1)
(adjacent wpc10r3 wpc9r3)
(adjacent wpc10r3 wpc10r2)
(adjacent wpc10r4 wpc9r4)
(adjacent wpc10r4 wpc10r3)
(adjacent wpc10r5 wpc9r5)
(adjacent wpc10r5 wpc10r4)
(adjacent wpc10r6 wpc9r6)
(adjacent wpc10r6 wpc10r5)
(adjacent wpc10r7 wpc9r7)
(adjacent wpc10r7 wpc10r6)
(adjacent wpc10r8 wpc9r8)
(adjacent wpc10r8 wpc10r7)
(adjacent wpc10r9 wpc9r9)
(adjacent wpc10r9 wpc10r8)
(adjacent wpc10r10 wpc9r10)
(adjacent wpc10r10 wpc10r9)
(adjacent wpc10r11 wpc9r11)
(adjacent wpc10r11 wpc10r10)
(adjacent wpc10r12 wpc9r12)
(adjacent wpc10r12 wpc10r11)
(adjacent wpc10r13 wpc9r13)
(adjacent wpc10r13 wpc10r12)
(adjacent wpc10r14 wpc9r14)
(adjacent wpc10r14 wpc10r13)
(adjacent wpc10r15 wpc9r15)
(adjacent wpc10r15 wpc10r14)
(adjacent wpc11r0 wpc10r0)
(adjacent wpc11r1 wpc10r1)
(adjacent wpc11r1 wpc11r0)
(adjacent wpc11r2 wpc10r2)
(adjacent wpc11r2 wpc11r1)
(adjacent wpc11r3 wpc10r3)
(adjacent wpc11r3 wpc11r2)
(adjacent wpc11r4 wpc10r4)
(adjacent wpc11r4 wpc11r3)
(adjacent wpc11r5 wpc10r5)
(adjacent wpc11r5 wpc11r4)
(adjacent wpc11r6 wpc10r6)
(adjacent wpc11r6 wpc11r5)
(adjacent wpc11r7 wpc10r7)
(adjacent wpc11r7 wpc11r6)
(adjacent wpc11r8 wpc10r8)
(adjacent wpc11r8 wpc11r7)
(adjacent wpc11r9 wpc10r9)
(adjacent wpc11r9 wpc11r8)
(adjacent wpc11r10 wpc10r10)
(adjacent wpc11r10 wpc11r9)
(adjacent wpc11r11 wpc10r11)
(adjacent wpc11r11 wpc11r10)
(adjacent wpc11r12 wpc10r12)
(adjacent wpc11r12 wpc11r11)
(adjacent wpc11r13 wpc10r13)
(adjacent wpc11r13 wpc11r12)
(adjacent wpc11r14 wpc10r14)
(adjacent wpc11r14 wpc11r13)
(adjacent wpc11r15 wpc10r15)
(adjacent wpc11r15 wpc11r14)
(adjacent wpc12r0 wpc11r0)
(adjacent wpc12r1 wpc11r1)
(adjacent wpc12r1 wpc12r0)
(adjacent wpc12r2 wpc11r2)
(adjacent wpc12r2 wpc12r1)
(adjacent wpc12r3 wpc11r3)
(adjacent wpc12r3 wpc12r2)
(adjacent wpc12r4 wpc11r4)
(adjacent wpc12r4 wpc12r3)
(adjacent wpc12r5 wpc11r5)
(adjacent wpc12r5 wpc12r4)
(adjacent wpc12r6 wpc11r6)
(adjacent wpc12r6 wpc12r5)
(adjacent wpc12r7 wpc11r7)
(adjacent wpc12r7 wpc12r6)
(adjacent wpc12r8 wpc11r8)
(adjacent wpc12r8 wpc12r7)
(adjacent wpc12r9 wpc11r9)
(adjacent wpc12r9 wpc12r8)
(adjacent wpc12r10 wpc11r10)
(adjacent wpc12r10 wpc12r9)
(adjacent wpc12r11 wpc11r11)
(adjacent wpc12r11 wpc12r10)
(adjacent wpc12r12 wpc11r12)
(adjacent wpc12r12 wpc12r11)
(adjacent wpc12r13 wpc11r13)
(adjacent wpc12r13 wpc12r12)
(adjacent wpc12r14 wpc11r14)
(adjacent wpc12r14 wpc12r13)
(adjacent wpc12r15 wpc11r15)
(adjacent wpc12r15 wpc12r14)
(adjacent wpc13r0 wpc12r0)
(adjacent wpc13r1 wpc12r1)
(adjacent wpc13r1 wpc13r0)
(adjacent wpc13r2 wpc12r2)
(adjacent wpc13r2 wpc13r1)
(adjacent wpc13r3 wpc12r3)
(adjacent wpc13r3 wpc13r2)
(adjacent wpc13r4 wpc12r4)
(adjacent wpc13r4 wpc13r3)
(adjacent wpc13r5 wpc12r5)
(adjacent wpc13r5 wpc13r4)
(adjacent wpc13r6 wpc12r6)
(adjacent wpc13r6 wpc13r5)
(adjacent wpc13r7 wpc12r7)
(adjacent wpc13r7 wpc13r6)
(adjacent wpc13r8 wpc12r8)
(adjacent wpc13r8 wpc13r7)
(adjacent wpc13r9 wpc12r9)
(adjacent wpc13r9 wpc13r8)
(adjacent wpc13r10 wpc12r10)
(adjacent wpc13r10 wpc13r9)
(adjacent wpc13r11 wpc12r11)
(adjacent wpc13r11 wpc13r10)
(adjacent wpc13r12 wpc12r12)
(adjacent wpc13r12 wpc13r11)
(adjacent wpc13r13 wpc12r13)
(adjacent wpc13r13 wpc13r12)
(adjacent wpc13r14 wpc12r14)
(adjacent wpc13r14 wpc13r13)
(adjacent wpc13r15 wpc12r15)
(adjacent wpc13r15 wpc13r14)
(adjacent wpc14r0 wpc13r0)
(adjacent wpc14r1 wpc13r1)
(adjacent wpc14r1 wpc14r0)
(adjacent wpc14r2 wpc13r2)
(adjacent wpc14r2 wpc14r1)
(adjacent wpc14r3 wpc13r3)
(adjacent wpc14r3 wpc14r2)
(adjacent wpc14r4 wpc13r4)
(adjacent wpc14r4 wpc14r3)
(adjacent wpc14r5 wpc13r5)
(adjacent wpc14r5 wpc14r4)
(adjacent wpc14r6 wpc13r6)
(adjacent wpc14r6 wpc14r5)
(adjacent wpc14r7 wpc13r7)
(adjacent wpc14r7 wpc14r6)
(adjacent wpc14r8 wpc13r8)
(adjacent wpc14r8 wpc14r7)
(adjacent wpc14r9 wpc13r9)
(adjacent wpc14r9 wpc14r8)
(adjacent wpc14r10 wpc13r10)
(adjacent wpc14r10 wpc14r9)
(adjacent wpc14r11 wpc13r11)
(adjacent wpc14r11 wpc14r10)
(adjacent wpc14r12 wpc13r12)
(adjacent wpc14r12 wpc14r11)
(adjacent wpc14r13 wpc13r13)
(adjacent wpc14r13 wpc14r12)
(adjacent wpc14r14 wpc13r14)
(adjacent wpc14r14 wpc14r13)
(adjacent wpc14r15 wpc13r15)
(adjacent wpc14r15 wpc14r14)
(adjacent wpc15r0 wpc14r0)
(adjacent wpc15r1 wpc14r1)
(adjacent wpc15r1 wpc15r0)
(adjacent wpc15r2 wpc14r2)
(adjacent wpc15r2 wpc15r1)
(adjacent wpc15r3 wpc14r3)
(adjacent wpc15r3 wpc15r2)
(adjacent wpc15r4 wpc14r4)
(adjacent wpc15r4 wpc15r3)
(adjacent wpc15r5 wpc14r5)
(adjacent wpc15r5 wpc15r4)
(adjacent wpc15r6 wpc14r6)
(adjacent wpc15r6 wpc15r5)
(adjacent wpc15r7 wpc14r7)
(adjacent wpc15r7 wpc15r6)
(adjacent wpc15r8 wpc14r8)
(adjacent wpc15r8 wpc15r7)
(adjacent wpc15r9 wpc14r9)
(adjacent wpc15r9 wpc15r8)
(adjacent wpc15r10 wpc14r10)
(adjacent wpc15r10 wpc15r9)
(adjacent wpc15r11 wpc14r11)
(adjacent wpc15r11 wpc15r10)
(adjacent wpc15r12 wpc14r12)
(adjacent wpc15r12 wpc15r11)
(adjacent wpc15r13 wpc14r13)
(adjacent wpc15r13 wpc15r12)
(adjacent wpc15r14 wpc14r14)
(adjacent wpc15r14 wpc15r13)
(adjacent wpc15r15 wpc14r15)
(adjacent wpc15r15 wpc15r14)

(isat robot wpc0r8)
 (isat ball wpc1r8)

)
(:goal 
    (isat ball wpc15r8)

)
)