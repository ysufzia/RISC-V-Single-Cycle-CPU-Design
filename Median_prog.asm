main:
    # Load array size from address 0x04
    addi t0, zero, 4        # t0 = 4
    lw s0, 0(t0)            # s0 = array_size (n)
    
    # Load array base address from address 0x08
    addi t0, zero, 8        # t0 = 8
    lw s1, 0(t0)            # s1 = base address of array
    
    # Setup loop variables
    addi s2, zero, 1        # s2 = current index (start at 1)
    addi s3, s0, -2         # s3 = n-2 (last index to process)
    
filter_loop:
    # Exit loop if index > n-2
    blt s3, s2, filter_done
    
    # Calculate pointer to current element: base + index*4
    add t0, s2, s2          # t0 = index * 2
    add t0, t0, t0          # t0 = index * 4
    add a0, s1, t0          # a0 = pointer to array[index]
    
    # Call median_of_three routine
    jal ra, median_of_three
    
    # Store median result back to array
    add t0, s2, s2          # t0 = index * 2
    add t0, t0, t0          # t0 = index * 4
    add t1, s1, t0          # t1 = address of array[index]
    sw a0, 0(t1)            # array[index] = median
    
    # Increment index and continue
    addi s2, s2, 1          # index++
    jal zero, filter_loop
    
filter_done:
    # Exit program
    addi a7, zero, 10       # System call code for exit
    ecall
median_of_three:
    # Load left element (array[i-1])
    addi t6, zero, -4       # t6 = -4
    add t6, a0, t6          # t6 = address of left element
    lw t0, 0(t6)            # t0 = left element
    
    # Load middle element (array[i])
    lw t1, 0(a0)            # t1 = middle element
    
    # Load right element (array[i+1])
    addi t6, zero, 4        # t6 = 4
    add t6, a0, t6          # t6 = address of right element
    lw t2, 0(t6)            # t2 = right element
    
    # Step 1: Find min(left, middle) and max(left, middle)
    blt t0, t1, left_is_smaller
    # left >= middle
    add t3, t1, zero        # t3 = min = middle
    add t4, t0, zero        # t4 = max = left
    jal zero, step2
    
left_is_smaller:
    # left < middle
    add t3, t0, zero        # t3 = min = left
    add t4, t1, zero        # t4 = max = middle
    
step2:
    # Step 2: Find min(max(left, middle), right)
    blt t4, t2, max_is_smaller
    # max >= right
    add t5, t2, zero        # t5 = min(max, right) = right
    jal zero, step3
    
max_is_smaller:
    # max < right
    add t5, t4, zero        # t5 = min(max, right) = max
    
step3:
    blt t3, t5, min_is_smaller
    # min(left, middle) >= min(max, right)
    add a0, t3, zero        # Return t3
    jalr zero, ra, 0        # Return to caller
    
min_is_smaller:
    # min(left, middle) < min(max, right)
    add a0, t5, zero        # Return t5
    jalr zero, ra, 0        # Return to caller
