###############################################################################
##	Author: Ecem Ilgun
##
##	_Lab2main - a program that calls linked list utility functions,
##		 depending on user selection.  _Lab2main outputs a 
##		message, then lists the menu options and get the user
##		selection, then calls the chosen routine, and repeats
##   
##	Total Register Usage:
##		$a0 - used for input arguments to syscalls and for passing the 
##		      pointer to the linked list to the utility functions
##		$s0 - keeps the size of linked list
##		$s1 - keeps the head node's address
##		$s2 - keeps the prev node (for iteration)
##		$s3 - keeps the curr node (for iteration)
##		$t1 - keeps integer input for menu
##		$t2 - keeps if the value is less than zero
##		$t3 - used to keep track of loop 
##			(int t3 = size; t3 > 0 ; t3--)
##		$t4 - keeps current node
##		$t5 - keeps current node's value
##		
##
##      linked list consists of 0 or more elements, in 
##		dynamic memory segment (i.e. heap)
##	elements of the linked list contain 2 parts:
##		at address z: pointerToNext element (unsigned integer), 4 bytes
##		at address z+4: value of the element (signed integer), 4 bytes
##
#########################################
#		text segment		#
#########################################
	.text		
 	
	.globl _Lab2main
 
	addi $a0, $0, 0
  	addi $s3, $0, 0
  	addi $s0, $0, 0
  	addi $s1, $0, 0 # $s1 is zero if no list is created yet
  	addi $s2, $0, 0
  	addi $t1, $0, 0
  	addi $t2, $0, 0
  	addi $t3, $0, 0
  	addi $t4, $0, 0
  	addi $t5, $0, 0
  	
 	
_Lab2main:
	addi $s1, $v0, 0 # Load previous return address to $s1
	
	la $a0, menu
	li $v0, 4 # Prints string
	syscall

	li $v0, 5 # Accepts an integer
	syscall
	
	add $t1, $0, $v0 # Stores integer($v0) in temporary var
	
	addi $a0, $s1, 0 # Load head pointer back to $a0 as argument
	beq $t1, 0, quit
	beq $t1, 1, create_list
	beq $t1, 2, display_list
	beq $t1, 3, insert_before_value
	beq $t1, 4, reverse_list
	

###################################################################
#### create_list - a linked list utility routine, 
##			which creates the contents, element 
##			by element, of a linked list
##
##	Register Usage:
##		$a0 - used for input arguments to syscalls and for passing the 
##		      pointer to the linked list to the utility functions
##		$s0 - keeps the number of elements in linked list
##		$s1 - keeps the head node's address
##		$s2 - keeps the prev node (for iteration)
##		$s3 - keeps the curr node (for iteration)
##		$t3 - used to keep track of loop 
##			(int t3 = size; t3 > 0 ; t3--)
###################################################################   
create_list:
	la $a0, endl
	li $v0, 4 # Prints string
	syscall
	
	# Get size #
	la $a0, sizePrompt
	li $v0, 4 # Prints string
	syscall
	
	li $v0, 5 # Accepts an integer
	syscall
	
	addi $s0, $v0, 0 # Store size in $s0
	slti $t7, $s0, 0 # if $s0 is negative
	beq $t7, 1, input_error
	beq $s0, 0, empty_list
	
	la $a0, inputPrompt 
	li $v0, 4 # Prints string
	syscall
	
	# For the first node #
	li $v0, 5 # Integer input
	syscall
	addi $a1, $v0, 0 #give input as argument
	jal create_node
	addi $s1, $v0, 0 # Keep the first node's address(return value of create_node) in $s1
	
	addi $s2, $s1, 0 # Give the first node as the previous node to its successors
		    	 # prev = $s2
	addi $t3, $s0, -1 # Loop through 
	j input_loop


##################################################################	
#### empty_list - Prints to the screen that list is empty
##		     and returns to main menu
##	
################################################################## 
empty_list:
	la $a0, emptyPrompt 
	li $v0, 4 # Prints string
	syscall
	
	addi $v0, $0, 0 # Return 0 as the output
	j back_to_main


##################################################################
#### create_node - allocates a space for the new node and asks 
##		        for a value
##	
##	Register Usage:
##		$s3 - cur = node's address 
##		$t2 - to check if the value is less than zero
################################################################## 
create_node:
	li $v0, 9 # Allocates heap memory,
	li $a0, 8 # for 8 bytes(32 bits).
	syscall
	
	addi $s3, $v0, 0 # Give the node's address to $s3
	
	slti $t2, $a1, 0	# If input is less than zero,
	beq $t2, 1, input_error # redirects to input_error routine.
	
	sw $0, 0($s3) # pointerToNext = null 
	sw $a1, 4($s3) # *value = *pointerToNext + 4 
	
	addi $v0, $s3, 0 # Return $v0 as the node's pointer
	jr $ra



##################################################################
#### input_loop - Loop for getting the integer values
##			of linked list	and adding them to tail
##			using add_node routine
##	
##	Register Usage:
##		$s3 - cur = node's address 
##		$s2 - prev
##		$t3 - used to keep track of loop 
##			(int t3 = size; t3 > 0 ; t3--)
################################################################## 
input_loop:
	beq $t3, 0, back_to_main

	li $v0, 5 # Integer input
	syscall
	addi $a1, $v0, 0 #give input as argument
	jal create_node
	addi $s3, $v0, 0 # Load return value (created node's pointer) to $s3(curr)
	
	# For the middle nodes, update pointerToNext of the previous node #
	sw $s3, 0($s2) # prev-> pointerToNext = curr
	addi $s2, $s3, 0 # prev = curr
	
	addi $t3, $t3, -1
	j input_loop
	

##################################################################
#### input_error - Rejects a negative input and returns back to main
################################################################## 
input_error:
	la $a0, negPrompt 
	li $v0, 4 # Prints string
	syscall
	
	addi $v0, $0, -1 # Return -1 as the error output
	j back_to_main

##################################################################
#### back_to_main - Returns to main
################################################################## 
back_to_main:
	addi $v0, $s1, 0 # Load first node's address back to $v0
	j _Lab2main


##################################################################
#### display_list - a linked list utility routine, 
##			which shows the contents, element 
##			by element, of a linked list
##	
##	Register Usage:
##		$a0 - used for input arguments to syscalls and for passing the 
##		      pointer to the linked list to the utility 
##		$t3 - used to keep track of loop 
##			(int t3 = size; t3 > 0 ; t3--)
##		$s0 - keeps the number of elements in linked list
##		$t4 - keeps current node
################################################################## 
display_list:
	addi $t4, $a0, 0 # cur($t4) = head node ($a0)
	addi $s1, $a0, 0 # Keep head node in $s1
	
	la $a0, endl
	li $v0, 4 # Print string
	syscall
	
	add $t3, $0, 1 # int $t3
	j display_loop


##################################################################
#### display_loop - runs through each element and prints them
##
##	Register Usage:
##		$t3 - used to keep track of loop 
##			(int t3 = size; t3 > 0 ; t3--)
##		$t4 - keeps current node
##		$t5 - keeps current node's value
##################################################################
display_loop:
	beq $t3, 0, exit_display
	
	lw $t3, 0($t4) # Load (pointerToNext) content of $t4 to $t3
	lw $t5, 4($t4) # Load (value) content of $t4 to $t5
	addi $a0, $t5, 0 # read value

	li $v0, 1 #Print value
	syscall 
	
	la $a0, comma
	li $v0, 4 # Print string
	syscall
	
	lw $t4, 0($t4)	# cur = cur -> pointerToNext
	j display_loop

##################################################################
#### exit_display - Exits displaying the linked list,
##			returns back to main menu
##
##################################################################
exit_display:
	la $a0, endl
	li $v0, 4 # Print string
	syscall
	
	la $a0, msg1
	li $v0, 4 # Print string
	syscall
	
	la $a0, endl
	li $v0, 4 # Print string
	syscall
	
	addi $v0, $s1, 0 #Return address
	j back_to_main

##################################################################
#### insert_before_value - Inserts a node to the beginning of the
##			linked list, returns the new head's address
##			is successful, returns -1 if insertion
##			if unsuccessful
##
##	Register Usage:
##		$a0 - used for input arguments to syscalls and for passing the 
##		      pointer to the linked list to the utility functions
##		$t6 - temporary register
##################################################################
insert_before_value:
	addi $t6, $a0, 0 # Load previous head node's address
	beq $t6, $0, return_error
	
	la $a0, inputPrompt 
	li $v0, 4 # Prints string
	syscall
	
	li $v0, 5 # Integer input
	syscall
	
	addi $a1, $v0, 0 #give input as argument
	j insert_to_head # returns new head in $v0

		
##################################################################
#### insert_to_head - Inserts a node to head of list
##################################################################
insert_to_head:
	# insert_to_head receives same $a1 argument as value input
	jal create_node # Creates the new head node with pointerToNext = nul (0)
			# This call will return -1 if a negative value is added
	addi $t4, $v0, 0 # $t4 now holds new node
	
	sw $t6, 0($t4) # pointerToNext of $t4 = $t6  
	
	addi $s1, $t4, 0
	addi $v0, $t4, 0 # Return new node
	
	j back_to_main

##################################################################
#### return_error - Exists with return -1
##################################################################
return_error:		
	addi $v0, $0, -1 #Return -1
	j back_to_main


##################################################################
#### reverse_list - Reverses the list, returns it's head pointer in $v0
##			
##			
##			
##
##	Register Usage:
##		$a0 - used for input arguments to syscalls and for passing the 
##		      pointer to the linked list to the utility functions
##		$s2 - keeps the prev node (for iteration)
##		$s3 - keeps the curr node (for iteration)
##################################################################
reverse_list:
	addi $t6, $a0, 0 # cur = head pointer
	beq $t6, $0, return_error
	
	lw $t3, 0($t6) # Load (pointerToNext) content of $t6 to $t3
	
	lw $a1, 4($t6) # Load (value) content of $t6 to $t5, as argument
	jal create_node # Creates the new head node with pointerToNext = nul (0)
			# This call will return -1 if a negative value is added
	addi $s2, $v0, 0 # Give the new node's address to $a0 ($s1 is new head for now!)
	
	#lw $t6, 0($t6)	# cur = cur -> pointerToNext
	j reverse_loop
	
	
##################################################################
#### reverse_list - Reverses the list, returns it's head pointer in $v0
##			
##	
##################################################################			
reverse_loop:
	lw $a1, 4($t3) # Load (value) content of $t3 to $a1, as argument
	jal create_node # Creates the new head node with pointerToNext = nul (0)
			# This call will return -1 if a negative value is added
	addi $s3, $v0, 0 # $s3 now holds new node
	
	sw $s2, 0($s3) # pointerToNext of $s3 = $s2
	
	addi $s1, $s3, 0
	addi $v0, $s3, 0 # Return new node
	
	lw $t3, 0($t3) # Load (pointerToNext) content of $t6 to $t3
	addi $s2, $s3, 0
	beq $t3, 0, back_to_main
	j reverse_loop



##################################################################
#### quit - Exists the program
##################################################################
quit:		
	li $v0, 10 # Terminate program
	syscall
	
	
#########################################
#     	 	data segment			#
#########################################
	 .data

msg1:		.asciiz "The linked list has been completely displayed. \n"
menu:		.asciiz " Press 1 to create a linked list \n Press 2 to display the linked list\n Press 3 to insert a new node before a certain value\n Press 4 to reverse the list\n Press 0 to quit the program\n"
endl:		.asciiz "\n"
inputPrompt:	.asciiz "Enter non-negative integer values: \n"
negPrompt:	.asciiz "Negative input received. (Returning to main)\n\n"
sizePrompt:	.asciiz "Enter the number of elements: \n"
comma:		.asciiz ", "
emptyPrompt:		.asciiz "Empty list, no list is created.\n\n"

##
## end of main
