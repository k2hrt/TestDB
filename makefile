# Makefile for TestDB project

TestDB: TestDB.o -lpq
	gcc TestDB.o -lpq -o TestDB
	
TestDB.o: TestDB.c
	gcc -c TestDB.c
	
