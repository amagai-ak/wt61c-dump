all: wt61c-dump

wt61c-dump: wt61c-dump.c
	gcc wt61c-dump.c -Wall -O2 -o wt61c-dump -lpthread

clean:
	rm -f wt61c-dump
