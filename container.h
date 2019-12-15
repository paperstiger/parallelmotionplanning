/*
Write memory efficient container that requires the minimum memory allocation
*/
#include "stdlib.h"

/*
A memory efficient list that requires minimum number of memory allocation.
We can initialize it with some integer N and memory containing N data are allocated.
The user can requesting memory for a new stuff, if not enough memory is left,
We allocate a new piece of memory.
*/
template<class T>
class List {
public:
    List *prev;  // in case we need more memory
    List *next;
    T *memory;
    int capacity;
    int cur_index;

    List(int size) : capacity(size), cur_index(0) {
        memory = (T*)malloc(sizeof(T) * size);
        prev = nullptr;
        next = nullptr;
    }

    std::pair<T*, List*> get_one() {
        if(cur_index < capacity) {
            cur_index += 1;
            return std::make_pair(memory + cur_index - 1, this);
        }
        List *new_list = new List(capacity);
        new_list->prev = this;
        this->next = new_list;
        return new_list->get_one();
    }

    List *insert(const T &val) {
        T* tmp;
        List *tmplist;
        std::tie(tmp, tmplist) = get_one();
        *tmp = val;
        return tmplist;
    }

    ~List() {
        free(memory);
    }
};

template<class T>
struct ListIterator {
    List<T> *lst = nullptr;
    int index = 0;

    void init(List<T> *list) {
        index = 0;
        lst = list;
    }

    // update to the next, return false if reached end
    bool next() {
        index++;
        if(index == lst->cur_index) {
            index = 0;
            lst = lst->next;
            if(lst == nullptr)
                return false;
        }
        return true;
    }

    bool is_valid() {
        if(!lst)
            return false;
        if(lst->next)
            return true;
        return index < lst->cur_index;
    }

    T get_val() {
        return lst->memory[index];
    }
};

template<class T>
void free_list(List<T> *bucket) {
    List<T>* prev;
    while(bucket != nullptr) {
        prev = bucket->prev;
        // free(bucket->memory);
        free(bucket);
        bucket = prev;
    }
}