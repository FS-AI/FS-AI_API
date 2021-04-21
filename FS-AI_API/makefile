# http://nuclear.mutantstargoat.com/articles/make/

CC = gcc
SRC = $(wildcard *.c)
OBJ = $(SRC:.c=.o)

CFLAGS = 
LDFLAGS = -lpthread

fs-ai_api.a: $(OBJ) $(CFLAGS) $(LDFLAGS)
	ar -rs $@ $^

.PHONY: clean
clean:
	rm -f $(OBJ) fs-ai_api.a
