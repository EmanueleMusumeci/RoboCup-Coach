(define (problem simplestriker)
    (:domain robocupdeterministic)
    (:objects ball robot wpc0r0 wpc0r1 wpc0r10 wpc0r11 wpc0r12 wpc0r13 wpc0r14 wpc0r15 wpc0r16 wpc0r17 wpc0r18 wpc0r19 wpc0r2 wpc0r20 wpc0r21 wpc0r22 wpc0r23 wpc0r24 wpc0r3 wpc0r4 wpc0r5 wpc0r6 wpc0r7 wpc0r8 wpc0r9 wpc10r0 wpc10r1 wpc10r10 wpc10r11 wpc10r12 wpc10r13 wpc10r14 wpc10r15 wpc10r16 wpc10r17 wpc10r18 wpc10r19 wpc10r2 wpc10r20 wpc10r21 wpc10r22 wpc10r23 wpc10r24 wpc10r3 wpc10r4 wpc10r5 wpc10r6 wpc10r7 wpc10r8 wpc10r9 wpc11r0 wpc11r1 wpc11r10 wpc11r11 wpc11r12 wpc11r13 wpc11r14 wpc11r15 wpc11r16 wpc11r17 wpc11r18 wpc11r19 wpc11r2 wpc11r20 wpc11r21 wpc11r22 wpc11r23 wpc11r24 wpc11r3 wpc11r4 wpc11r5 wpc11r6 wpc11r7 wpc11r8 wpc11r9 wpc12r0 wpc12r1 wpc12r10 wpc12r11 wpc12r12 wpc12r13 wpc12r14 wpc12r15 wpc12r16 wpc12r17 wpc12r18 wpc12r19 wpc12r2 wpc12r20 wpc12r21 wpc12r22 wpc12r23 wpc12r24 wpc12r3 wpc12r4 wpc12r5 wpc12r6 wpc12r7 wpc12r8 wpc12r9 wpc13r0 wpc13r1 wpc13r10 wpc13r11 wpc13r12 wpc13r13 wpc13r14 wpc13r15 wpc13r16 wpc13r17 wpc13r18 wpc13r19 wpc13r2 wpc13r20 wpc13r21 wpc13r22 wpc13r23 wpc13r24 wpc13r3 wpc13r4 wpc13r5 wpc13r6 wpc13r7 wpc13r8 wpc13r9 wpc14r0 wpc14r1 wpc14r10 wpc14r11 wpc14r12 wpc14r13 wpc14r14 wpc14r15 wpc14r16 wpc14r17 wpc14r18 wpc14r19 wpc14r2 wpc14r20 wpc14r21 wpc14r22 wpc14r23 wpc14r24 wpc14r3 wpc14r4 wpc14r5 wpc14r6 wpc14r7 wpc14r8 wpc14r9 wpc15r0 wpc15r1 wpc15r10 wpc15r11 wpc15r12 wpc15r13 wpc15r14 wpc15r15 wpc15r16 wpc15r17 wpc15r18 wpc15r19 wpc15r2 wpc15r20 wpc15r21 wpc15r22 wpc15r23 wpc15r24 wpc15r3 wpc15r4 wpc15r5 wpc15r6 wpc15r7 wpc15r8 wpc15r9 wpc16r0 wpc16r1 wpc16r10 wpc16r11 wpc16r12 wpc16r13 wpc16r14 wpc16r15 wpc16r16 wpc16r17 wpc16r18 wpc16r19 wpc16r2 wpc16r20 wpc16r21 wpc16r22 wpc16r23 wpc16r24 wpc16r3 wpc16r4 wpc16r5 wpc16r6 wpc16r7 wpc16r8 wpc16r9 wpc17r0 wpc17r1 wpc17r10 wpc17r11 wpc17r12 wpc17r13 wpc17r14 wpc17r15 wpc17r16 wpc17r17 wpc17r18 wpc17r19 wpc17r2 wpc17r20 wpc17r21 wpc17r22 wpc17r23 wpc17r24 wpc17r3 wpc17r4 wpc17r5 wpc17r6 wpc17r7 wpc17r8 wpc17r9 wpc18r0 wpc18r1 wpc18r10 wpc18r11 wpc18r12 wpc18r13 wpc18r14 wpc18r15 wpc18r16 wpc18r17 wpc18r18 wpc18r19 wpc18r2 wpc18r20 wpc18r21 wpc18r22 wpc18r23 wpc18r24 wpc18r3 wpc18r4 wpc18r5 wpc18r6 wpc18r7 wpc18r8 wpc18r9 wpc19r0 wpc19r1 wpc19r10 wpc19r11 wpc19r12 wpc19r13 wpc19r14 wpc19r15 wpc19r16 wpc19r17 wpc19r18 wpc19r19 wpc19r2 wpc19r20 wpc19r21 wpc19r22 wpc19r23 wpc19r24 wpc19r3 wpc19r4 wpc19r5 wpc19r6 wpc19r7 wpc19r8 wpc19r9 wpc1r0 wpc1r1 wpc1r10 wpc1r11 wpc1r12 wpc1r13 wpc1r14 wpc1r15 wpc1r16 wpc1r17 wpc1r18 wpc1r19 wpc1r2 wpc1r20 wpc1r21 wpc1r22 wpc1r23 wpc1r24 wpc1r3 wpc1r4 wpc1r5 wpc1r6 wpc1r7 wpc1r8 wpc1r9 wpc20r0 wpc20r1 wpc20r10 wpc20r11 wpc20r12 wpc20r13 wpc20r14 wpc20r15 wpc20r16 wpc20r17 wpc20r18 wpc20r19 wpc20r2 wpc20r20 wpc20r21 wpc20r22 wpc20r23 wpc20r24 wpc20r3 wpc20r4 wpc20r5 wpc20r6 wpc20r7 wpc20r8 wpc20r9 wpc21r0 wpc21r1 wpc21r10 wpc21r11 wpc21r12 wpc21r13 wpc21r14 wpc21r15 wpc21r16 wpc21r17 wpc21r18 wpc21r19 wpc21r2 wpc21r20 wpc21r21 wpc21r22 wpc21r23 wpc21r24 wpc21r3 wpc21r4 wpc21r5 wpc21r6 wpc21r7 wpc21r8 wpc21r9 wpc22r0 wpc22r1 wpc22r10 wpc22r11 wpc22r12 wpc22r13 wpc22r14 wpc22r15 wpc22r16 wpc22r17 wpc22r18 wpc22r19 wpc22r2 wpc22r20 wpc22r21 wpc22r22 wpc22r23 wpc22r24 wpc22r3 wpc22r4 wpc22r5 wpc22r6 wpc22r7 wpc22r8 wpc22r9 wpc23r0 wpc23r1 wpc23r10 wpc23r11 wpc23r12 wpc23r13 wpc23r14 wpc23r15 wpc23r16 wpc23r17 wpc23r18 wpc23r19 wpc23r2 wpc23r20 wpc23r21 wpc23r22 wpc23r23 wpc23r24 wpc23r3 wpc23r4 wpc23r5 wpc23r6 wpc23r7 wpc23r8 wpc23r9 wpc24r0 wpc24r1 wpc24r10 wpc24r11 wpc24r12 wpc24r13 wpc24r14 wpc24r15 wpc24r16 wpc24r17 wpc24r18 wpc24r19 wpc24r2 wpc24r20 wpc24r21 wpc24r22 wpc24r23 wpc24r24 wpc24r3 wpc24r4 wpc24r5 wpc24r6 wpc24r7 wpc24r8 wpc24r9 wpc2r0 wpc2r1 wpc2r10 wpc2r11 wpc2r12 wpc2r13 wpc2r14 wpc2r15 wpc2r16 wpc2r17 wpc2r18 wpc2r19 wpc2r2 wpc2r20 wpc2r21 wpc2r22 wpc2r23 wpc2r24 wpc2r3 wpc2r4 wpc2r5 wpc2r6 wpc2r7 wpc2r8 wpc2r9 wpc3r0 wpc3r1 wpc3r10 wpc3r11 wpc3r12 wpc3r13 wpc3r14 wpc3r15 wpc3r16 wpc3r17 wpc3r18 wpc3r19 wpc3r2 wpc3r20 wpc3r21 wpc3r22 wpc3r23 wpc3r24 wpc3r3 wpc3r4 wpc3r5 wpc3r6 wpc3r7 wpc3r8 wpc3r9 wpc4r0 wpc4r1 wpc4r10 wpc4r11 wpc4r12 wpc4r13 wpc4r14 wpc4r15 wpc4r16 wpc4r17 wpc4r18 wpc4r19 wpc4r2 wpc4r20 wpc4r21 wpc4r22 wpc4r23 wpc4r24 wpc4r3 wpc4r4 wpc4r5 wpc4r6 wpc4r7 wpc4r8 wpc4r9 wpc5r0 wpc5r1 wpc5r10 wpc5r11 wpc5r12 wpc5r13 wpc5r14 wpc5r15 wpc5r16 wpc5r17 wpc5r18 wpc5r19 wpc5r2 wpc5r20 wpc5r21 wpc5r22 wpc5r23 wpc5r24 wpc5r3 wpc5r4 wpc5r5 wpc5r6 wpc5r7 wpc5r8 wpc5r9 wpc6r0 wpc6r1 wpc6r10 wpc6r11 wpc6r12 wpc6r13 wpc6r14 wpc6r15 wpc6r16 wpc6r17 wpc6r18 wpc6r19 wpc6r2 wpc6r20 wpc6r21 wpc6r22 wpc6r23 wpc6r24 wpc6r3 wpc6r4 wpc6r5 wpc6r6 wpc6r7 wpc6r8 wpc6r9 wpc7r0 wpc7r1 wpc7r10 wpc7r11 wpc7r12 wpc7r13 wpc7r14 wpc7r15 wpc7r16 wpc7r17 wpc7r18 wpc7r19 wpc7r2 wpc7r20 wpc7r21 wpc7r22 wpc7r23 wpc7r24 wpc7r3 wpc7r4 wpc7r5 wpc7r6 wpc7r7 wpc7r8 wpc7r9 wpc8r0 wpc8r1 wpc8r10 wpc8r11 wpc8r12 wpc8r13 wpc8r14 wpc8r15 wpc8r16 wpc8r17 wpc8r18 wpc8r19 wpc8r2 wpc8r20 wpc8r21 wpc8r22 wpc8r23 wpc8r24 wpc8r3 wpc8r4 wpc8r5 wpc8r6 wpc8r7 wpc8r8 wpc8r9 wpc9r0 wpc9r1 wpc9r10 wpc9r11 wpc9r12 wpc9r13 wpc9r14 wpc9r15 wpc9r16 wpc9r17 wpc9r18 wpc9r19 wpc9r2 wpc9r20 wpc9r21 wpc9r22 wpc9r23 wpc9r24 wpc9r3 wpc9r4 wpc9r5 wpc9r6 wpc9r7 wpc9r8 wpc9r9)
    (:init (adjacent wpc0r1 wpc0r0) (adjacent wpc0r10 wpc0r9) (adjacent wpc0r11 wpc0r10) (adjacent wpc0r12 wpc0r11) (adjacent wpc0r13 wpc0r12) (adjacent wpc0r14 wpc0r13) (adjacent wpc0r15 wpc0r14) (adjacent wpc0r16 wpc0r15) (adjacent wpc0r17 wpc0r16) (adjacent wpc0r18 wpc0r17) (adjacent wpc0r19 wpc0r18) (adjacent wpc0r2 wpc0r1) (adjacent wpc0r20 wpc0r19) (adjacent wpc0r21 wpc0r20) (adjacent wpc0r22 wpc0r21) (adjacent wpc0r23 wpc0r22) (adjacent wpc0r24 wpc0r23) (adjacent wpc0r3 wpc0r2) (adjacent wpc0r4 wpc0r3) (adjacent wpc0r5 wpc0r4) (adjacent wpc0r6 wpc0r5) (adjacent wpc0r7 wpc0r6) (adjacent wpc0r8 wpc0r7) (adjacent wpc0r9 wpc0r8) (adjacent wpc10r0 wpc9r0) (adjacent wpc10r1 wpc10r0) (adjacent wpc10r1 wpc9r1) (adjacent wpc10r10 wpc10r9) (adjacent wpc10r10 wpc9r10) (adjacent wpc10r11 wpc10r10) (adjacent wpc10r11 wpc9r11) (adjacent wpc10r12 wpc10r11) (adjacent wpc10r12 wpc9r12) (adjacent wpc10r13 wpc10r12) (adjacent wpc10r13 wpc9r13) (adjacent wpc10r14 wpc10r13) (adjacent wpc10r14 wpc9r14) (adjacent wpc10r15 wpc10r14) (adjacent wpc10r15 wpc9r15) (adjacent wpc10r16 wpc10r15) (adjacent wpc10r16 wpc9r16) (adjacent wpc10r17 wpc10r16) (adjacent wpc10r17 wpc9r17) (adjacent wpc10r18 wpc10r17) (adjacent wpc10r18 wpc9r18) (adjacent wpc10r19 wpc10r18) (adjacent wpc10r19 wpc9r19) (adjacent wpc10r2 wpc10r1) (adjacent wpc10r2 wpc9r2) (adjacent wpc10r20 wpc10r19) (adjacent wpc10r20 wpc9r20) (adjacent wpc10r21 wpc10r20) (adjacent wpc10r21 wpc9r21) (adjacent wpc10r22 wpc10r21) (adjacent wpc10r22 wpc9r22) (adjacent wpc10r23 wpc10r22) (adjacent wpc10r23 wpc9r23) (adjacent wpc10r24 wpc10r23) (adjacent wpc10r24 wpc9r24) (adjacent wpc10r3 wpc10r2) (adjacent wpc10r3 wpc9r3) (adjacent wpc10r4 wpc10r3) (adjacent wpc10r4 wpc9r4) (adjacent wpc10r5 wpc10r4) (adjacent wpc10r5 wpc9r5) (adjacent wpc10r6 wpc10r5) (adjacent wpc10r6 wpc9r6) (adjacent wpc10r7 wpc10r6) (adjacent wpc10r7 wpc9r7) (adjacent wpc10r8 wpc10r7) (adjacent wpc10r8 wpc9r8) (adjacent wpc10r9 wpc10r8) (adjacent wpc10r9 wpc9r9) (adjacent wpc11r0 wpc10r0) (adjacent wpc11r1 wpc10r1) (adjacent wpc11r1 wpc11r0) (adjacent wpc11r10 wpc10r10) (adjacent wpc11r10 wpc11r9) (adjacent wpc11r11 wpc10r11) (adjacent wpc11r11 wpc11r10) (adjacent wpc11r12 wpc10r12) (adjacent wpc11r12 wpc11r11) (adjacent wpc11r13 wpc10r13) (adjacent wpc11r13 wpc11r12) (adjacent wpc11r14 wpc10r14) (adjacent wpc11r14 wpc11r13) (adjacent wpc11r15 wpc10r15) (adjacent wpc11r15 wpc11r14) (adjacent wpc11r16 wpc10r16) (adjacent wpc11r16 wpc11r15) (adjacent wpc11r17 wpc10r17) (adjacent wpc11r17 wpc11r16) (adjacent wpc11r18 wpc10r18) (adjacent wpc11r18 wpc11r17) (adjacent wpc11r19 wpc10r19) (adjacent wpc11r19 wpc11r18) (adjacent wpc11r2 wpc10r2) (adjacent wpc11r2 wpc11r1) (adjacent wpc11r20 wpc10r20) (adjacent wpc11r20 wpc11r19) (adjacent wpc11r21 wpc10r21) (adjacent wpc11r21 wpc11r20) (adjacent wpc11r22 wpc10r22) (adjacent wpc11r22 wpc11r21) (adjacent wpc11r23 wpc10r23) (adjacent wpc11r23 wpc11r22) (adjacent wpc11r24 wpc10r24) (adjacent wpc11r24 wpc11r23) (adjacent wpc11r3 wpc10r3) (adjacent wpc11r3 wpc11r2) (adjacent wpc11r4 wpc10r4) (adjacent wpc11r4 wpc11r3) (adjacent wpc11r5 wpc10r5) (adjacent wpc11r5 wpc11r4) (adjacent wpc11r6 wpc10r6) (adjacent wpc11r6 wpc11r5) (adjacent wpc11r7 wpc10r7) (adjacent wpc11r7 wpc11r6) (adjacent wpc11r8 wpc10r8) (adjacent wpc11r8 wpc11r7) (adjacent wpc11r9 wpc10r9) (adjacent wpc11r9 wpc11r8) (adjacent wpc12r0 wpc11r0) (adjacent wpc12r1 wpc11r1) (adjacent wpc12r1 wpc12r0) (adjacent wpc12r10 wpc11r10) (adjacent wpc12r10 wpc12r9) (adjacent wpc12r11 wpc11r11) (adjacent wpc12r11 wpc12r10) (adjacent wpc12r12 wpc11r12) (adjacent wpc12r12 wpc12r11) (adjacent wpc12r13 wpc11r13) (adjacent wpc12r13 wpc12r12) (adjacent wpc12r14 wpc11r14) (adjacent wpc12r14 wpc12r13) (adjacent wpc12r15 wpc11r15) (adjacent wpc12r15 wpc12r14) (adjacent wpc12r16 wpc11r16) (adjacent wpc12r16 wpc12r15) (adjacent wpc12r17 wpc11r17) (adjacent wpc12r17 wpc12r16) (adjacent wpc12r18 wpc11r18) (adjacent wpc12r18 wpc12r17) (adjacent wpc12r19 wpc11r19) (adjacent wpc12r19 wpc12r18) (adjacent wpc12r2 wpc11r2) (adjacent wpc12r2 wpc12r1) (adjacent wpc12r20 wpc11r20) (adjacent wpc12r20 wpc12r19) (adjacent wpc12r21 wpc11r21) (adjacent wpc12r21 wpc12r20) (adjacent wpc12r22 wpc11r22) (adjacent wpc12r22 wpc12r21) (adjacent wpc12r23 wpc11r23) (adjacent wpc12r23 wpc12r22) (adjacent wpc12r24 wpc11r24) (adjacent wpc12r24 wpc12r23) (adjacent wpc12r3 wpc11r3) (adjacent wpc12r3 wpc12r2) (adjacent wpc12r4 wpc11r4) (adjacent wpc12r4 wpc12r3) (adjacent wpc12r5 wpc11r5) (adjacent wpc12r5 wpc12r4) (adjacent wpc12r6 wpc11r6) (adjacent wpc12r6 wpc12r5) (adjacent wpc12r7 wpc11r7) (adjacent wpc12r7 wpc12r6) (adjacent wpc12r8 wpc11r8) (adjacent wpc12r8 wpc12r7) (adjacent wpc12r9 wpc11r9) (adjacent wpc12r9 wpc12r8) (adjacent wpc13r0 wpc12r0) (adjacent wpc13r1 wpc12r1) (adjacent wpc13r1 wpc13r0) (adjacent wpc13r10 wpc12r10) (adjacent wpc13r10 wpc13r9) (adjacent wpc13r11 wpc12r11) (adjacent wpc13r11 wpc13r10) (adjacent wpc13r12 wpc12r12) (adjacent wpc13r12 wpc13r11) (adjacent wpc13r13 wpc12r13) (adjacent wpc13r13 wpc13r12) (adjacent wpc13r14 wpc12r14) (adjacent wpc13r14 wpc13r13) (adjacent wpc13r15 wpc12r15) (adjacent wpc13r15 wpc13r14) (adjacent wpc13r16 wpc12r16) (adjacent wpc13r16 wpc13r15) (adjacent wpc13r17 wpc12r17) (adjacent wpc13r17 wpc13r16) (adjacent wpc13r18 wpc12r18) (adjacent wpc13r18 wpc13r17) (adjacent wpc13r19 wpc12r19) (adjacent wpc13r19 wpc13r18) (adjacent wpc13r2 wpc12r2) (adjacent wpc13r2 wpc13r1) (adjacent wpc13r20 wpc12r20) (adjacent wpc13r20 wpc13r19) (adjacent wpc13r21 wpc12r21) (adjacent wpc13r21 wpc13r20) (adjacent wpc13r22 wpc12r22) (adjacent wpc13r22 wpc13r21) (adjacent wpc13r23 wpc12r23) (adjacent wpc13r23 wpc13r22) (adjacent wpc13r24 wpc12r24) (adjacent wpc13r24 wpc13r23) (adjacent wpc13r3 wpc12r3) (adjacent wpc13r3 wpc13r2) (adjacent wpc13r4 wpc12r4) (adjacent wpc13r4 wpc13r3) (adjacent wpc13r5 wpc12r5) (adjacent wpc13r5 wpc13r4) (adjacent wpc13r6 wpc12r6) (adjacent wpc13r6 wpc13r5) (adjacent wpc13r7 wpc12r7) (adjacent wpc13r7 wpc13r6) (adjacent wpc13r8 wpc12r8) (adjacent wpc13r8 wpc13r7) (adjacent wpc13r9 wpc12r9) (adjacent wpc13r9 wpc13r8) (adjacent wpc14r0 wpc13r0) (adjacent wpc14r1 wpc13r1) (adjacent wpc14r1 wpc14r0) (adjacent wpc14r10 wpc13r10) (adjacent wpc14r10 wpc14r9) (adjacent wpc14r11 wpc13r11) (adjacent wpc14r11 wpc14r10) (adjacent wpc14r12 wpc13r12) (adjacent wpc14r12 wpc14r11) (adjacent wpc14r13 wpc13r13) (adjacent wpc14r13 wpc14r12) (adjacent wpc14r14 wpc13r14) (adjacent wpc14r14 wpc14r13) (adjacent wpc14r15 wpc13r15) (adjacent wpc14r15 wpc14r14) (adjacent wpc14r16 wpc13r16) (adjacent wpc14r16 wpc14r15) (adjacent wpc14r17 wpc13r17) (adjacent wpc14r17 wpc14r16) (adjacent wpc14r18 wpc13r18) (adjacent wpc14r18 wpc14r17) (adjacent wpc14r19 wpc13r19) (adjacent wpc14r19 wpc14r18) (adjacent wpc14r2 wpc13r2) (adjacent wpc14r2 wpc14r1) (adjacent wpc14r20 wpc13r20) (adjacent wpc14r20 wpc14r19) (adjacent wpc14r21 wpc13r21) (adjacent wpc14r21 wpc14r20) (adjacent wpc14r22 wpc13r22) (adjacent wpc14r22 wpc14r21) (adjacent wpc14r23 wpc13r23) (adjacent wpc14r23 wpc14r22) (adjacent wpc14r24 wpc13r24) (adjacent wpc14r24 wpc14r23) (adjacent wpc14r3 wpc13r3) (adjacent wpc14r3 wpc14r2) (adjacent wpc14r4 wpc13r4) (adjacent wpc14r4 wpc14r3) (adjacent wpc14r5 wpc13r5) (adjacent wpc14r5 wpc14r4) (adjacent wpc14r6 wpc13r6) (adjacent wpc14r6 wpc14r5) (adjacent wpc14r7 wpc13r7) (adjacent wpc14r7 wpc14r6) (adjacent wpc14r8 wpc13r8) (adjacent wpc14r8 wpc14r7) (adjacent wpc14r9 wpc13r9) (adjacent wpc14r9 wpc14r8) (adjacent wpc15r0 wpc14r0) (adjacent wpc15r1 wpc14r1) (adjacent wpc15r1 wpc15r0) (adjacent wpc15r10 wpc14r10) (adjacent wpc15r10 wpc15r9) (adjacent wpc15r11 wpc14r11) (adjacent wpc15r11 wpc15r10) (adjacent wpc15r12 wpc14r12) (adjacent wpc15r12 wpc15r11) (adjacent wpc15r13 wpc14r13) (adjacent wpc15r13 wpc15r12) (adjacent wpc15r14 wpc14r14) (adjacent wpc15r14 wpc15r13) (adjacent wpc15r15 wpc14r15) (adjacent wpc15r15 wpc15r14) (adjacent wpc15r16 wpc14r16) (adjacent wpc15r16 wpc15r15) (adjacent wpc15r17 wpc14r17) (adjacent wpc15r17 wpc15r16) (adjacent wpc15r18 wpc14r18) (adjacent wpc15r18 wpc15r17) (adjacent wpc15r19 wpc14r19) (adjacent wpc15r19 wpc15r18) (adjacent wpc15r2 wpc14r2) (adjacent wpc15r2 wpc15r1) (adjacent wpc15r20 wpc14r20) (adjacent wpc15r20 wpc15r19) (adjacent wpc15r21 wpc14r21) (adjacent wpc15r21 wpc15r20) (adjacent wpc15r22 wpc14r22) (adjacent wpc15r22 wpc15r21) (adjacent wpc15r23 wpc14r23) (adjacent wpc15r23 wpc15r22) (adjacent wpc15r24 wpc14r24) (adjacent wpc15r24 wpc15r23) (adjacent wpc15r3 wpc14r3) (adjacent wpc15r3 wpc15r2) (adjacent wpc15r4 wpc14r4) (adjacent wpc15r4 wpc15r3) (adjacent wpc15r5 wpc14r5) (adjacent wpc15r5 wpc15r4) (adjacent wpc15r6 wpc14r6) (adjacent wpc15r6 wpc15r5) (adjacent wpc15r7 wpc14r7) (adjacent wpc15r7 wpc15r6) (adjacent wpc15r8 wpc14r8) (adjacent wpc15r8 wpc15r7) (adjacent wpc15r9 wpc14r9) (adjacent wpc15r9 wpc15r8) (adjacent wpc16r0 wpc15r0) (adjacent wpc16r1 wpc15r1) (adjacent wpc16r1 wpc16r0) (adjacent wpc16r10 wpc15r10) (adjacent wpc16r10 wpc16r9) (adjacent wpc16r11 wpc15r11) (adjacent wpc16r11 wpc16r10) (adjacent wpc16r12 wpc15r12) (adjacent wpc16r12 wpc16r11) (adjacent wpc16r13 wpc15r13) (adjacent wpc16r13 wpc16r12) (adjacent wpc16r14 wpc15r14) (adjacent wpc16r14 wpc16r13) (adjacent wpc16r15 wpc15r15) (adjacent wpc16r15 wpc16r14) (adjacent wpc16r16 wpc15r16) (adjacent wpc16r16 wpc16r15) (adjacent wpc16r17 wpc15r17) (adjacent wpc16r17 wpc16r16) (adjacent wpc16r18 wpc15r18) (adjacent wpc16r18 wpc16r17) (adjacent wpc16r19 wpc15r19) (adjacent wpc16r19 wpc16r18) (adjacent wpc16r2 wpc15r2) (adjacent wpc16r2 wpc16r1) (adjacent wpc16r20 wpc15r20) (adjacent wpc16r20 wpc16r19) (adjacent wpc16r21 wpc15r21) (adjacent wpc16r21 wpc16r20) (adjacent wpc16r22 wpc15r22) (adjacent wpc16r22 wpc16r21) (adjacent wpc16r23 wpc15r23) (adjacent wpc16r23 wpc16r22) (adjacent wpc16r24 wpc15r24) (adjacent wpc16r24 wpc16r23) (adjacent wpc16r3 wpc15r3) (adjacent wpc16r3 wpc16r2) (adjacent wpc16r4 wpc15r4) (adjacent wpc16r4 wpc16r3) (adjacent wpc16r5 wpc15r5) (adjacent wpc16r5 wpc16r4) (adjacent wpc16r6 wpc15r6) (adjacent wpc16r6 wpc16r5) (adjacent wpc16r7 wpc15r7) (adjacent wpc16r7 wpc16r6) (adjacent wpc16r8 wpc15r8) (adjacent wpc16r8 wpc16r7) (adjacent wpc16r9 wpc15r9) (adjacent wpc16r9 wpc16r8) (adjacent wpc17r0 wpc16r0) (adjacent wpc17r1 wpc16r1) (adjacent wpc17r1 wpc17r0) (adjacent wpc17r10 wpc16r10) (adjacent wpc17r10 wpc17r9) (adjacent wpc17r11 wpc16r11) (adjacent wpc17r11 wpc17r10) (adjacent wpc17r12 wpc16r12) (adjacent wpc17r12 wpc17r11) (adjacent wpc17r13 wpc16r13) (adjacent wpc17r13 wpc17r12) (adjacent wpc17r14 wpc16r14) (adjacent wpc17r14 wpc17r13) (adjacent wpc17r15 wpc16r15) (adjacent wpc17r15 wpc17r14) (adjacent wpc17r16 wpc16r16) (adjacent wpc17r16 wpc17r15) (adjacent wpc17r17 wpc16r17) (adjacent wpc17r17 wpc17r16) (adjacent wpc17r18 wpc16r18) (adjacent wpc17r18 wpc17r17) (adjacent wpc17r19 wpc16r19) (adjacent wpc17r19 wpc17r18) (adjacent wpc17r2 wpc16r2) (adjacent wpc17r2 wpc17r1) (adjacent wpc17r20 wpc16r20) (adjacent wpc17r20 wpc17r19) (adjacent wpc17r21 wpc16r21) (adjacent wpc17r21 wpc17r20) (adjacent wpc17r22 wpc16r22) (adjacent wpc17r22 wpc17r21) (adjacent wpc17r23 wpc16r23) (adjacent wpc17r23 wpc17r22) (adjacent wpc17r24 wpc16r24) (adjacent wpc17r24 wpc17r23) (adjacent wpc17r3 wpc16r3) (adjacent wpc17r3 wpc17r2) (adjacent wpc17r4 wpc16r4) (adjacent wpc17r4 wpc17r3) (adjacent wpc17r5 wpc16r5) (adjacent wpc17r5 wpc17r4) (adjacent wpc17r6 wpc16r6) (adjacent wpc17r6 wpc17r5) (adjacent wpc17r7 wpc16r7) (adjacent wpc17r7 wpc17r6) (adjacent wpc17r8 wpc16r8) (adjacent wpc17r8 wpc17r7) (adjacent wpc17r9 wpc16r9) (adjacent wpc17r9 wpc17r8) (adjacent wpc18r0 wpc17r0) (adjacent wpc18r1 wpc17r1) (adjacent wpc18r1 wpc18r0) (adjacent wpc18r10 wpc17r10) (adjacent wpc18r10 wpc18r9) (adjacent wpc18r11 wpc17r11) (adjacent wpc18r11 wpc18r10) (adjacent wpc18r12 wpc17r12) (adjacent wpc18r12 wpc18r11) (adjacent wpc18r13 wpc17r13) (adjacent wpc18r13 wpc18r12) (adjacent wpc18r14 wpc17r14) (adjacent wpc18r14 wpc18r13) (adjacent wpc18r15 wpc17r15) (adjacent wpc18r15 wpc18r14) (adjacent wpc18r16 wpc17r16) (adjacent wpc18r16 wpc18r15) (adjacent wpc18r17 wpc17r17) (adjacent wpc18r17 wpc18r16) (adjacent wpc18r18 wpc17r18) (adjacent wpc18r18 wpc18r17) (adjacent wpc18r19 wpc17r19) (adjacent wpc18r19 wpc18r18) (adjacent wpc18r2 wpc17r2) (adjacent wpc18r2 wpc18r1) (adjacent wpc18r20 wpc17r20) (adjacent wpc18r20 wpc18r19) (adjacent wpc18r21 wpc17r21) (adjacent wpc18r21 wpc18r20) (adjacent wpc18r22 wpc17r22) (adjacent wpc18r22 wpc18r21) (adjacent wpc18r23 wpc17r23) (adjacent wpc18r23 wpc18r22) (adjacent wpc18r24 wpc17r24) (adjacent wpc18r24 wpc18r23) (adjacent wpc18r3 wpc17r3) (adjacent wpc18r3 wpc18r2) (adjacent wpc18r4 wpc17r4) (adjacent wpc18r4 wpc18r3) (adjacent wpc18r5 wpc17r5) (adjacent wpc18r5 wpc18r4) (adjacent wpc18r6 wpc17r6) (adjacent wpc18r6 wpc18r5) (adjacent wpc18r7 wpc17r7) (adjacent wpc18r7 wpc18r6) (adjacent wpc18r8 wpc17r8) (adjacent wpc18r8 wpc18r7) (adjacent wpc18r9 wpc17r9) (adjacent wpc18r9 wpc18r8) (adjacent wpc19r0 wpc18r0) (adjacent wpc19r1 wpc18r1) (adjacent wpc19r1 wpc19r0) (adjacent wpc19r10 wpc18r10) (adjacent wpc19r10 wpc19r9) (adjacent wpc19r11 wpc18r11) (adjacent wpc19r11 wpc19r10) (adjacent wpc19r12 wpc18r12) (adjacent wpc19r12 wpc19r11) (adjacent wpc19r13 wpc18r13) (adjacent wpc19r13 wpc19r12) (adjacent wpc19r14 wpc18r14) (adjacent wpc19r14 wpc19r13) (adjacent wpc19r15 wpc18r15) (adjacent wpc19r15 wpc19r14) (adjacent wpc19r16 wpc18r16) (adjacent wpc19r16 wpc19r15) (adjacent wpc19r17 wpc18r17) (adjacent wpc19r17 wpc19r16) (adjacent wpc19r18 wpc18r18) (adjacent wpc19r18 wpc19r17) (adjacent wpc19r19 wpc18r19) (adjacent wpc19r19 wpc19r18) (adjacent wpc19r2 wpc18r2) (adjacent wpc19r2 wpc19r1) (adjacent wpc19r20 wpc18r20) (adjacent wpc19r20 wpc19r19) (adjacent wpc19r21 wpc18r21) (adjacent wpc19r21 wpc19r20) (adjacent wpc19r22 wpc18r22) (adjacent wpc19r22 wpc19r21) (adjacent wpc19r23 wpc18r23) (adjacent wpc19r23 wpc19r22) (adjacent wpc19r24 wpc18r24) (adjacent wpc19r24 wpc19r23) (adjacent wpc19r3 wpc18r3) (adjacent wpc19r3 wpc19r2) (adjacent wpc19r4 wpc18r4) (adjacent wpc19r4 wpc19r3) (adjacent wpc19r5 wpc18r5) (adjacent wpc19r5 wpc19r4) (adjacent wpc19r6 wpc18r6) (adjacent wpc19r6 wpc19r5) (adjacent wpc19r7 wpc18r7) (adjacent wpc19r7 wpc19r6) (adjacent wpc19r8 wpc18r8) (adjacent wpc19r8 wpc19r7) (adjacent wpc19r9 wpc18r9) (adjacent wpc19r9 wpc19r8) (adjacent wpc1r0 wpc0r0) (adjacent wpc1r1 wpc0r1) (adjacent wpc1r1 wpc1r0) (adjacent wpc1r10 wpc0r10) (adjacent wpc1r10 wpc1r9) (adjacent wpc1r11 wpc0r11) (adjacent wpc1r11 wpc1r10) (adjacent wpc1r12 wpc0r12) (adjacent wpc1r12 wpc1r11) (adjacent wpc1r13 wpc0r13) (adjacent wpc1r13 wpc1r12) (adjacent wpc1r14 wpc0r14) (adjacent wpc1r14 wpc1r13) (adjacent wpc1r15 wpc0r15) (adjacent wpc1r15 wpc1r14) (adjacent wpc1r16 wpc0r16) (adjacent wpc1r16 wpc1r15) (adjacent wpc1r17 wpc0r17) (adjacent wpc1r17 wpc1r16) (adjacent wpc1r18 wpc0r18) (adjacent wpc1r18 wpc1r17) (adjacent wpc1r19 wpc0r19) (adjacent wpc1r19 wpc1r18) (adjacent wpc1r2 wpc0r2) (adjacent wpc1r2 wpc1r1) (adjacent wpc1r20 wpc0r20) (adjacent wpc1r20 wpc1r19) (adjacent wpc1r21 wpc0r21) (adjacent wpc1r21 wpc1r20) (adjacent wpc1r22 wpc0r22) (adjacent wpc1r22 wpc1r21) (adjacent wpc1r23 wpc0r23) (adjacent wpc1r23 wpc1r22) (adjacent wpc1r24 wpc0r24) (adjacent wpc1r24 wpc1r23) (adjacent wpc1r3 wpc0r3) (adjacent wpc1r3 wpc1r2) (adjacent wpc1r4 wpc0r4) (adjacent wpc1r4 wpc1r3) (adjacent wpc1r5 wpc0r5) (adjacent wpc1r5 wpc1r4) (adjacent wpc1r6 wpc0r6) (adjacent wpc1r6 wpc1r5) (adjacent wpc1r7 wpc0r7) (adjacent wpc1r7 wpc1r6) (adjacent wpc1r8 wpc0r8) (adjacent wpc1r8 wpc1r7) (adjacent wpc1r9 wpc0r9) (adjacent wpc1r9 wpc1r8) (adjacent wpc20r0 wpc19r0) (adjacent wpc20r1 wpc19r1) (adjacent wpc20r1 wpc20r0) (adjacent wpc20r10 wpc19r10) (adjacent wpc20r10 wpc20r9) (adjacent wpc20r11 wpc19r11) (adjacent wpc20r11 wpc20r10) (adjacent wpc20r12 wpc19r12) (adjacent wpc20r12 wpc20r11) (adjacent wpc20r13 wpc19r13) (adjacent wpc20r13 wpc20r12) (adjacent wpc20r14 wpc19r14) (adjacent wpc20r14 wpc20r13) (adjacent wpc20r15 wpc19r15) (adjacent wpc20r15 wpc20r14) (adjacent wpc20r16 wpc19r16) (adjacent wpc20r16 wpc20r15) (adjacent wpc20r17 wpc19r17) (adjacent wpc20r17 wpc20r16) (adjacent wpc20r18 wpc19r18) (adjacent wpc20r18 wpc20r17) (adjacent wpc20r19 wpc19r19) (adjacent wpc20r19 wpc20r18) (adjacent wpc20r2 wpc19r2) (adjacent wpc20r2 wpc20r1) (adjacent wpc20r20 wpc19r20) (adjacent wpc20r20 wpc20r19) (adjacent wpc20r21 wpc19r21) (adjacent wpc20r21 wpc20r20) (adjacent wpc20r22 wpc19r22) (adjacent wpc20r22 wpc20r21) (adjacent wpc20r23 wpc19r23) (adjacent wpc20r23 wpc20r22) (adjacent wpc20r24 wpc19r24) (adjacent wpc20r24 wpc20r23) (adjacent wpc20r3 wpc19r3) (adjacent wpc20r3 wpc20r2) (adjacent wpc20r4 wpc19r4) (adjacent wpc20r4 wpc20r3) (adjacent wpc20r5 wpc19r5) (adjacent wpc20r5 wpc20r4) (adjacent wpc20r6 wpc19r6) (adjacent wpc20r6 wpc20r5) (adjacent wpc20r7 wpc19r7) (adjacent wpc20r7 wpc20r6) (adjacent wpc20r8 wpc19r8) (adjacent wpc20r8 wpc20r7) (adjacent wpc20r9 wpc19r9) (adjacent wpc20r9 wpc20r8) (adjacent wpc21r0 wpc20r0) (adjacent wpc21r1 wpc20r1) (adjacent wpc21r1 wpc21r0) (adjacent wpc21r10 wpc20r10) (adjacent wpc21r10 wpc21r9) (adjacent wpc21r11 wpc20r11) (adjacent wpc21r11 wpc21r10) (adjacent wpc21r12 wpc20r12) (adjacent wpc21r12 wpc21r11) (adjacent wpc21r13 wpc20r13) (adjacent wpc21r13 wpc21r12) (adjacent wpc21r14 wpc20r14) (adjacent wpc21r14 wpc21r13) (adjacent wpc21r15 wpc20r15) (adjacent wpc21r15 wpc21r14) (adjacent wpc21r16 wpc20r16) (adjacent wpc21r16 wpc21r15) (adjacent wpc21r17 wpc20r17) (adjacent wpc21r17 wpc21r16) (adjacent wpc21r18 wpc20r18) (adjacent wpc21r18 wpc21r17) (adjacent wpc21r19 wpc20r19) (adjacent wpc21r19 wpc21r18) (adjacent wpc21r2 wpc20r2) (adjacent wpc21r2 wpc21r1) (adjacent wpc21r20 wpc20r20) (adjacent wpc21r20 wpc21r19) (adjacent wpc21r21 wpc20r21) (adjacent wpc21r21 wpc21r20) (adjacent wpc21r22 wpc20r22) (adjacent wpc21r22 wpc21r21) (adjacent wpc21r23 wpc20r23) (adjacent wpc21r23 wpc21r22) (adjacent wpc21r24 wpc20r24) (adjacent wpc21r24 wpc21r23) (adjacent wpc21r3 wpc20r3) (adjacent wpc21r3 wpc21r2) (adjacent wpc21r4 wpc20r4) (adjacent wpc21r4 wpc21r3) (adjacent wpc21r5 wpc20r5) (adjacent wpc21r5 wpc21r4) (adjacent wpc21r6 wpc20r6) (adjacent wpc21r6 wpc21r5) (adjacent wpc21r7 wpc20r7) (adjacent wpc21r7 wpc21r6) (adjacent wpc21r8 wpc20r8) (adjacent wpc21r8 wpc21r7) (adjacent wpc21r9 wpc20r9) (adjacent wpc21r9 wpc21r8) (adjacent wpc22r0 wpc21r0) (adjacent wpc22r1 wpc21r1) (adjacent wpc22r1 wpc22r0) (adjacent wpc22r10 wpc21r10) (adjacent wpc22r10 wpc22r9) (adjacent wpc22r11 wpc21r11) (adjacent wpc22r11 wpc22r10) (adjacent wpc22r12 wpc21r12) (adjacent wpc22r12 wpc22r11) (adjacent wpc22r13 wpc21r13) (adjacent wpc22r13 wpc22r12) (adjacent wpc22r14 wpc21r14) (adjacent wpc22r14 wpc22r13) (adjacent wpc22r15 wpc21r15) (adjacent wpc22r15 wpc22r14) (adjacent wpc22r16 wpc21r16) (adjacent wpc22r16 wpc22r15) (adjacent wpc22r17 wpc21r17) (adjacent wpc22r17 wpc22r16) (adjacent wpc22r18 wpc21r18) (adjacent wpc22r18 wpc22r17) (adjacent wpc22r19 wpc21r19) (adjacent wpc22r19 wpc22r18) (adjacent wpc22r2 wpc21r2) (adjacent wpc22r2 wpc22r1) (adjacent wpc22r20 wpc21r20) (adjacent wpc22r20 wpc22r19) (adjacent wpc22r21 wpc21r21) (adjacent wpc22r21 wpc22r20) (adjacent wpc22r22 wpc21r22) (adjacent wpc22r22 wpc22r21) (adjacent wpc22r23 wpc21r23) (adjacent wpc22r23 wpc22r22) (adjacent wpc22r24 wpc21r24) (adjacent wpc22r24 wpc22r23) (adjacent wpc22r3 wpc21r3) (adjacent wpc22r3 wpc22r2) (adjacent wpc22r4 wpc21r4) (adjacent wpc22r4 wpc22r3) (adjacent wpc22r5 wpc21r5) (adjacent wpc22r5 wpc22r4) (adjacent wpc22r6 wpc21r6) (adjacent wpc22r6 wpc22r5) (adjacent wpc22r7 wpc21r7) (adjacent wpc22r7 wpc22r6) (adjacent wpc22r8 wpc21r8) (adjacent wpc22r8 wpc22r7) (adjacent wpc22r9 wpc21r9) (adjacent wpc22r9 wpc22r8) (adjacent wpc23r0 wpc22r0) (adjacent wpc23r1 wpc22r1) (adjacent wpc23r1 wpc23r0) (adjacent wpc23r10 wpc22r10) (adjacent wpc23r10 wpc23r9) (adjacent wpc23r11 wpc22r11) (adjacent wpc23r11 wpc23r10) (adjacent wpc23r12 wpc22r12) (adjacent wpc23r12 wpc23r11) (adjacent wpc23r13 wpc22r13) (adjacent wpc23r13 wpc23r12) (adjacent wpc23r14 wpc22r14) (adjacent wpc23r14 wpc23r13) (adjacent wpc23r15 wpc22r15) (adjacent wpc23r15 wpc23r14) (adjacent wpc23r16 wpc22r16) (adjacent wpc23r16 wpc23r15) (adjacent wpc23r17 wpc22r17) (adjacent wpc23r17 wpc23r16) (adjacent wpc23r18 wpc22r18) (adjacent wpc23r18 wpc23r17) (adjacent wpc23r19 wpc22r19) (adjacent wpc23r19 wpc23r18) (adjacent wpc23r2 wpc22r2) (adjacent wpc23r2 wpc23r1) (adjacent wpc23r20 wpc22r20) (adjacent wpc23r20 wpc23r19) (adjacent wpc23r21 wpc22r21) (adjacent wpc23r21 wpc23r20) (adjacent wpc23r22 wpc22r22) (adjacent wpc23r22 wpc23r21) (adjacent wpc23r23 wpc22r23) (adjacent wpc23r23 wpc23r22) (adjacent wpc23r24 wpc22r24) (adjacent wpc23r24 wpc23r23) (adjacent wpc23r3 wpc22r3) (adjacent wpc23r3 wpc23r2) (adjacent wpc23r4 wpc22r4) (adjacent wpc23r4 wpc23r3) (adjacent wpc23r5 wpc22r5) (adjacent wpc23r5 wpc23r4) (adjacent wpc23r6 wpc22r6) (adjacent wpc23r6 wpc23r5) (adjacent wpc23r7 wpc22r7) (adjacent wpc23r7 wpc23r6) (adjacent wpc23r8 wpc22r8) (adjacent wpc23r8 wpc23r7) (adjacent wpc23r9 wpc22r9) (adjacent wpc23r9 wpc23r8) (adjacent wpc24r0 wpc23r0) (adjacent wpc24r1 wpc23r1) (adjacent wpc24r1 wpc24r0) (adjacent wpc24r10 wpc23r10) (adjacent wpc24r10 wpc24r9) (adjacent wpc24r11 wpc23r11) (adjacent wpc24r11 wpc24r10) (adjacent wpc24r12 wpc23r12) (adjacent wpc24r12 wpc24r11) (adjacent wpc24r13 wpc23r13) (adjacent wpc24r13 wpc24r12) (adjacent wpc24r14 wpc23r14) (adjacent wpc24r14 wpc24r13) (adjacent wpc24r15 wpc23r15) (adjacent wpc24r15 wpc24r14) (adjacent wpc24r16 wpc23r16) (adjacent wpc24r16 wpc24r15) (adjacent wpc24r17 wpc23r17) (adjacent wpc24r17 wpc24r16) (adjacent wpc24r18 wpc23r18) (adjacent wpc24r18 wpc24r17) (adjacent wpc24r19 wpc23r19) (adjacent wpc24r19 wpc24r18) (adjacent wpc24r2 wpc23r2) (adjacent wpc24r2 wpc24r1) (adjacent wpc24r20 wpc23r20) (adjacent wpc24r20 wpc24r19) (adjacent wpc24r21 wpc23r21) (adjacent wpc24r21 wpc24r20) (adjacent wpc24r22 wpc23r22) (adjacent wpc24r22 wpc24r21) (adjacent wpc24r23 wpc23r23) (adjacent wpc24r23 wpc24r22) (adjacent wpc24r24 wpc23r24) (adjacent wpc24r24 wpc24r23) (adjacent wpc24r3 wpc23r3) (adjacent wpc24r3 wpc24r2) (adjacent wpc24r4 wpc23r4) (adjacent wpc24r4 wpc24r3) (adjacent wpc24r5 wpc23r5) (adjacent wpc24r5 wpc24r4) (adjacent wpc24r6 wpc23r6) (adjacent wpc24r6 wpc24r5) (adjacent wpc24r7 wpc23r7) (adjacent wpc24r7 wpc24r6) (adjacent wpc24r8 wpc23r8) (adjacent wpc24r8 wpc24r7) (adjacent wpc24r9 wpc23r9) (adjacent wpc24r9 wpc24r8) (adjacent wpc2r0 wpc1r0) (adjacent wpc2r1 wpc1r1) (adjacent wpc2r1 wpc2r0) (adjacent wpc2r10 wpc1r10) (adjacent wpc2r10 wpc2r9) (adjacent wpc2r11 wpc1r11) (adjacent wpc2r11 wpc2r10) (adjacent wpc2r12 wpc1r12) (adjacent wpc2r12 wpc2r11) (adjacent wpc2r13 wpc1r13) (adjacent wpc2r13 wpc2r12) (adjacent wpc2r14 wpc1r14) (adjacent wpc2r14 wpc2r13) (adjacent wpc2r15 wpc1r15) (adjacent wpc2r15 wpc2r14) (adjacent wpc2r16 wpc1r16) (adjacent wpc2r16 wpc2r15) (adjacent wpc2r17 wpc1r17) (adjacent wpc2r17 wpc2r16) (adjacent wpc2r18 wpc1r18) (adjacent wpc2r18 wpc2r17) (adjacent wpc2r19 wpc1r19) (adjacent wpc2r19 wpc2r18) (adjacent wpc2r2 wpc1r2) (adjacent wpc2r2 wpc2r1) (adjacent wpc2r20 wpc1r20) (adjacent wpc2r20 wpc2r19) (adjacent wpc2r21 wpc1r21) (adjacent wpc2r21 wpc2r20) (adjacent wpc2r22 wpc1r22) (adjacent wpc2r22 wpc2r21) (adjacent wpc2r23 wpc1r23) (adjacent wpc2r23 wpc2r22) (adjacent wpc2r24 wpc1r24) (adjacent wpc2r24 wpc2r23) (adjacent wpc2r3 wpc1r3) (adjacent wpc2r3 wpc2r2) (adjacent wpc2r4 wpc1r4) (adjacent wpc2r4 wpc2r3) (adjacent wpc2r5 wpc1r5) (adjacent wpc2r5 wpc2r4) (adjacent wpc2r6 wpc1r6) (adjacent wpc2r6 wpc2r5) (adjacent wpc2r7 wpc1r7) (adjacent wpc2r7 wpc2r6) (adjacent wpc2r8 wpc1r8) (adjacent wpc2r8 wpc2r7) (adjacent wpc2r9 wpc1r9) (adjacent wpc2r9 wpc2r8) (adjacent wpc3r0 wpc2r0) (adjacent wpc3r1 wpc2r1) (adjacent wpc3r1 wpc3r0) (adjacent wpc3r10 wpc2r10) (adjacent wpc3r10 wpc3r9) (adjacent wpc3r11 wpc2r11) (adjacent wpc3r11 wpc3r10) (adjacent wpc3r12 wpc2r12) (adjacent wpc3r12 wpc3r11) (adjacent wpc3r13 wpc2r13) (adjacent wpc3r13 wpc3r12) (adjacent wpc3r14 wpc2r14) (adjacent wpc3r14 wpc3r13) (adjacent wpc3r15 wpc2r15) (adjacent wpc3r15 wpc3r14) (adjacent wpc3r16 wpc2r16) (adjacent wpc3r16 wpc3r15) (adjacent wpc3r17 wpc2r17) (adjacent wpc3r17 wpc3r16) (adjacent wpc3r18 wpc2r18) (adjacent wpc3r18 wpc3r17) (adjacent wpc3r19 wpc2r19) (adjacent wpc3r19 wpc3r18) (adjacent wpc3r2 wpc2r2) (adjacent wpc3r2 wpc3r1) (adjacent wpc3r20 wpc2r20) (adjacent wpc3r20 wpc3r19) (adjacent wpc3r21 wpc2r21) (adjacent wpc3r21 wpc3r20) (adjacent wpc3r22 wpc2r22) (adjacent wpc3r22 wpc3r21) (adjacent wpc3r23 wpc2r23) (adjacent wpc3r23 wpc3r22) (adjacent wpc3r24 wpc2r24) (adjacent wpc3r24 wpc3r23) (adjacent wpc3r3 wpc2r3) (adjacent wpc3r3 wpc3r2) (adjacent wpc3r4 wpc2r4) (adjacent wpc3r4 wpc3r3) (adjacent wpc3r5 wpc2r5) (adjacent wpc3r5 wpc3r4) (adjacent wpc3r6 wpc2r6) (adjacent wpc3r6 wpc3r5) (adjacent wpc3r7 wpc2r7) (adjacent wpc3r7 wpc3r6) (adjacent wpc3r8 wpc2r8) (adjacent wpc3r8 wpc3r7) (adjacent wpc3r9 wpc2r9) (adjacent wpc3r9 wpc3r8) (adjacent wpc4r0 wpc3r0) (adjacent wpc4r1 wpc3r1) (adjacent wpc4r1 wpc4r0) (adjacent wpc4r10 wpc3r10) (adjacent wpc4r10 wpc4r9) (adjacent wpc4r11 wpc3r11) (adjacent wpc4r11 wpc4r10) (adjacent wpc4r12 wpc3r12) (adjacent wpc4r12 wpc4r11) (adjacent wpc4r13 wpc3r13) (adjacent wpc4r13 wpc4r12) (adjacent wpc4r14 wpc3r14) (adjacent wpc4r14 wpc4r13) (adjacent wpc4r15 wpc3r15) (adjacent wpc4r15 wpc4r14) (adjacent wpc4r16 wpc3r16) (adjacent wpc4r16 wpc4r15) (adjacent wpc4r17 wpc3r17) (adjacent wpc4r17 wpc4r16) (adjacent wpc4r18 wpc3r18) (adjacent wpc4r18 wpc4r17) (adjacent wpc4r19 wpc3r19) (adjacent wpc4r19 wpc4r18) (adjacent wpc4r2 wpc3r2) (adjacent wpc4r2 wpc4r1) (adjacent wpc4r20 wpc3r20) (adjacent wpc4r20 wpc4r19) (adjacent wpc4r21 wpc3r21) (adjacent wpc4r21 wpc4r20) (adjacent wpc4r22 wpc3r22) (adjacent wpc4r22 wpc4r21) (adjacent wpc4r23 wpc3r23) (adjacent wpc4r23 wpc4r22) (adjacent wpc4r24 wpc3r24) (adjacent wpc4r24 wpc4r23) (adjacent wpc4r3 wpc3r3) (adjacent wpc4r3 wpc4r2) (adjacent wpc4r4 wpc3r4) (adjacent wpc4r4 wpc4r3) (adjacent wpc4r5 wpc3r5) (adjacent wpc4r5 wpc4r4) (adjacent wpc4r6 wpc3r6) (adjacent wpc4r6 wpc4r5) (adjacent wpc4r7 wpc3r7) (adjacent wpc4r7 wpc4r6) (adjacent wpc4r8 wpc3r8) (adjacent wpc4r8 wpc4r7) (adjacent wpc4r9 wpc3r9) (adjacent wpc4r9 wpc4r8) (adjacent wpc5r0 wpc4r0) (adjacent wpc5r1 wpc4r1) (adjacent wpc5r1 wpc5r0) (adjacent wpc5r10 wpc4r10) (adjacent wpc5r10 wpc5r9) (adjacent wpc5r11 wpc4r11) (adjacent wpc5r11 wpc5r10) (adjacent wpc5r12 wpc4r12) (adjacent wpc5r12 wpc5r11) (adjacent wpc5r13 wpc4r13) (adjacent wpc5r13 wpc5r12) (adjacent wpc5r14 wpc4r14) (adjacent wpc5r14 wpc5r13) (adjacent wpc5r15 wpc4r15) (adjacent wpc5r15 wpc5r14) (adjacent wpc5r16 wpc4r16) (adjacent wpc5r16 wpc5r15) (adjacent wpc5r17 wpc4r17) (adjacent wpc5r17 wpc5r16) (adjacent wpc5r18 wpc4r18) (adjacent wpc5r18 wpc5r17) (adjacent wpc5r19 wpc4r19) (adjacent wpc5r19 wpc5r18) (adjacent wpc5r2 wpc4r2) (adjacent wpc5r2 wpc5r1) (adjacent wpc5r20 wpc4r20) (adjacent wpc5r20 wpc5r19) (adjacent wpc5r21 wpc4r21) (adjacent wpc5r21 wpc5r20) (adjacent wpc5r22 wpc4r22) (adjacent wpc5r22 wpc5r21) (adjacent wpc5r23 wpc4r23) (adjacent wpc5r23 wpc5r22) (adjacent wpc5r24 wpc4r24) (adjacent wpc5r24 wpc5r23) (adjacent wpc5r3 wpc4r3) (adjacent wpc5r3 wpc5r2) (adjacent wpc5r4 wpc4r4) (adjacent wpc5r4 wpc5r3) (adjacent wpc5r5 wpc4r5) (adjacent wpc5r5 wpc5r4) (adjacent wpc5r6 wpc4r6) (adjacent wpc5r6 wpc5r5) (adjacent wpc5r7 wpc4r7) (adjacent wpc5r7 wpc5r6) (adjacent wpc5r8 wpc4r8) (adjacent wpc5r8 wpc5r7) (adjacent wpc5r9 wpc4r9) (adjacent wpc5r9 wpc5r8) (adjacent wpc6r0 wpc5r0) (adjacent wpc6r1 wpc5r1) (adjacent wpc6r1 wpc6r0) (adjacent wpc6r10 wpc5r10) (adjacent wpc6r10 wpc6r9) (adjacent wpc6r11 wpc5r11) (adjacent wpc6r11 wpc6r10) (adjacent wpc6r12 wpc5r12) (adjacent wpc6r12 wpc6r11) (adjacent wpc6r13 wpc5r13) (adjacent wpc6r13 wpc6r12) (adjacent wpc6r14 wpc5r14) (adjacent wpc6r14 wpc6r13) (adjacent wpc6r15 wpc5r15) (adjacent wpc6r15 wpc6r14) (adjacent wpc6r16 wpc5r16) (adjacent wpc6r16 wpc6r15) (adjacent wpc6r17 wpc5r17) (adjacent wpc6r17 wpc6r16) (adjacent wpc6r18 wpc5r18) (adjacent wpc6r18 wpc6r17) (adjacent wpc6r19 wpc5r19) (adjacent wpc6r19 wpc6r18) (adjacent wpc6r2 wpc5r2) (adjacent wpc6r2 wpc6r1) (adjacent wpc6r20 wpc5r20) (adjacent wpc6r20 wpc6r19) (adjacent wpc6r21 wpc5r21) (adjacent wpc6r21 wpc6r20) (adjacent wpc6r22 wpc5r22) (adjacent wpc6r22 wpc6r21) (adjacent wpc6r23 wpc5r23) (adjacent wpc6r23 wpc6r22) (adjacent wpc6r24 wpc5r24) (adjacent wpc6r24 wpc6r23) (adjacent wpc6r3 wpc5r3) (adjacent wpc6r3 wpc6r2) (adjacent wpc6r4 wpc5r4) (adjacent wpc6r4 wpc6r3) (adjacent wpc6r5 wpc5r5) (adjacent wpc6r5 wpc6r4) (adjacent wpc6r6 wpc5r6) (adjacent wpc6r6 wpc6r5) (adjacent wpc6r7 wpc5r7) (adjacent wpc6r7 wpc6r6) (adjacent wpc6r8 wpc5r8) (adjacent wpc6r8 wpc6r7) (adjacent wpc6r9 wpc5r9) (adjacent wpc6r9 wpc6r8) (adjacent wpc7r0 wpc6r0) (adjacent wpc7r1 wpc6r1) (adjacent wpc7r1 wpc7r0) (adjacent wpc7r10 wpc6r10) (adjacent wpc7r10 wpc7r9) (adjacent wpc7r11 wpc6r11) (adjacent wpc7r11 wpc7r10) (adjacent wpc7r12 wpc6r12) (adjacent wpc7r12 wpc7r11) (adjacent wpc7r13 wpc6r13) (adjacent wpc7r13 wpc7r12) (adjacent wpc7r14 wpc6r14) (adjacent wpc7r14 wpc7r13) (adjacent wpc7r15 wpc6r15) (adjacent wpc7r15 wpc7r14) (adjacent wpc7r16 wpc6r16) (adjacent wpc7r16 wpc7r15) (adjacent wpc7r17 wpc6r17) (adjacent wpc7r17 wpc7r16) (adjacent wpc7r18 wpc6r18) (adjacent wpc7r18 wpc7r17) (adjacent wpc7r19 wpc6r19) (adjacent wpc7r19 wpc7r18) (adjacent wpc7r2 wpc6r2) (adjacent wpc7r2 wpc7r1) (adjacent wpc7r20 wpc6r20) (adjacent wpc7r20 wpc7r19) (adjacent wpc7r21 wpc6r21) (adjacent wpc7r21 wpc7r20) (adjacent wpc7r22 wpc6r22) (adjacent wpc7r22 wpc7r21) (adjacent wpc7r23 wpc6r23) (adjacent wpc7r23 wpc7r22) (adjacent wpc7r24 wpc6r24) (adjacent wpc7r24 wpc7r23) (adjacent wpc7r3 wpc6r3) (adjacent wpc7r3 wpc7r2) (adjacent wpc7r4 wpc6r4) (adjacent wpc7r4 wpc7r3) (adjacent wpc7r5 wpc6r5) (adjacent wpc7r5 wpc7r4) (adjacent wpc7r6 wpc6r6) (adjacent wpc7r6 wpc7r5) (adjacent wpc7r7 wpc6r7) (adjacent wpc7r7 wpc7r6) (adjacent wpc7r8 wpc6r8) (adjacent wpc7r8 wpc7r7) (adjacent wpc7r9 wpc6r9) (adjacent wpc7r9 wpc7r8) (adjacent wpc8r0 wpc7r0) (adjacent wpc8r1 wpc7r1) (adjacent wpc8r1 wpc8r0) (adjacent wpc8r10 wpc7r10) (adjacent wpc8r10 wpc8r9) (adjacent wpc8r11 wpc7r11) (adjacent wpc8r11 wpc8r10) (adjacent wpc8r12 wpc7r12) (adjacent wpc8r12 wpc8r11) (adjacent wpc8r13 wpc7r13) (adjacent wpc8r13 wpc8r12) (adjacent wpc8r14 wpc7r14) (adjacent wpc8r14 wpc8r13) (adjacent wpc8r15 wpc7r15) (adjacent wpc8r15 wpc8r14) (adjacent wpc8r16 wpc7r16) (adjacent wpc8r16 wpc8r15) (adjacent wpc8r17 wpc7r17) (adjacent wpc8r17 wpc8r16) (adjacent wpc8r18 wpc7r18) (adjacent wpc8r18 wpc8r17) (adjacent wpc8r19 wpc7r19) (adjacent wpc8r19 wpc8r18) (adjacent wpc8r2 wpc7r2) (adjacent wpc8r2 wpc8r1) (adjacent wpc8r20 wpc7r20) (adjacent wpc8r20 wpc8r19) (adjacent wpc8r21 wpc7r21) (adjacent wpc8r21 wpc8r20) (adjacent wpc8r22 wpc7r22) (adjacent wpc8r22 wpc8r21) (adjacent wpc8r23 wpc7r23) (adjacent wpc8r23 wpc8r22) (adjacent wpc8r24 wpc7r24) (adjacent wpc8r24 wpc8r23) (adjacent wpc8r3 wpc7r3) (adjacent wpc8r3 wpc8r2) (adjacent wpc8r4 wpc7r4) (adjacent wpc8r4 wpc8r3) (adjacent wpc8r5 wpc7r5) (adjacent wpc8r5 wpc8r4) (adjacent wpc8r6 wpc7r6) (adjacent wpc8r6 wpc8r5) (adjacent wpc8r7 wpc7r7) (adjacent wpc8r7 wpc8r6) (adjacent wpc8r8 wpc7r8) (adjacent wpc8r8 wpc8r7) (adjacent wpc8r9 wpc7r9) (adjacent wpc8r9 wpc8r8) (adjacent wpc9r0 wpc8r0) (adjacent wpc9r1 wpc8r1) (adjacent wpc9r1 wpc9r0) (adjacent wpc9r10 wpc8r10) (adjacent wpc9r10 wpc9r9) (adjacent wpc9r11 wpc8r11) (adjacent wpc9r11 wpc9r10) (adjacent wpc9r12 wpc8r12) (adjacent wpc9r12 wpc9r11) (adjacent wpc9r13 wpc8r13) (adjacent wpc9r13 wpc9r12) (adjacent wpc9r14 wpc8r14) (adjacent wpc9r14 wpc9r13) (adjacent wpc9r15 wpc8r15) (adjacent wpc9r15 wpc9r14) (adjacent wpc9r16 wpc8r16) (adjacent wpc9r16 wpc9r15) (adjacent wpc9r17 wpc8r17) (adjacent wpc9r17 wpc9r16) (adjacent wpc9r18 wpc8r18) (adjacent wpc9r18 wpc9r17) (adjacent wpc9r19 wpc8r19) (adjacent wpc9r19 wpc9r18) (adjacent wpc9r2 wpc8r2) (adjacent wpc9r2 wpc9r1) (adjacent wpc9r20 wpc8r20) (adjacent wpc9r20 wpc9r19) (adjacent wpc9r21 wpc8r21) (adjacent wpc9r21 wpc9r20) (adjacent wpc9r22 wpc8r22) (adjacent wpc9r22 wpc9r21) (adjacent wpc9r23 wpc8r23) (adjacent wpc9r23 wpc9r22) (adjacent wpc9r24 wpc8r24) (adjacent wpc9r24 wpc9r23) (adjacent wpc9r3 wpc8r3) (adjacent wpc9r3 wpc9r2) (adjacent wpc9r4 wpc8r4) (adjacent wpc9r4 wpc9r3) (adjacent wpc9r5 wpc8r5) (adjacent wpc9r5 wpc9r4) (adjacent wpc9r6 wpc8r6) (adjacent wpc9r6 wpc9r5) (adjacent wpc9r7 wpc8r7) (adjacent wpc9r7 wpc9r6) (adjacent wpc9r8 wpc8r8) (adjacent wpc9r8 wpc9r7) (adjacent wpc9r9 wpc8r9) (adjacent wpc9r9 wpc9r8) (isat ball wpc1r12) (isat robot wpc0r12) (isball ball) (isrobot robot))
    (:goal (val_isat_ball_wpc24r12-and-Oisat_robot_wpc12r12))
)