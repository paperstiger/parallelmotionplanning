// Include these files.
#include <iostream>
#include <thread>
#include <vector>
#include <atomic>
#include <stdio.h>

using namespace std;

template<class T>
class Node{
public:
    T data;
    Node *next;

    Node(const T &dat) {
        data = dat;
        next = nullptr;
    }
};

template<class T>
class deque{
public:
    typedef Node<T> MyNode;
    std::atomic<MyNode*> head, tail;

    void push_right(const T& data) {
        MyNode *new_node = new MyNode(data);
        while(true) {
            MyNode *old_head = head.load(std::memory_order_relaxed), *old_tail = tail.load(std::memory_order_relaxed);
            if(!old_head) {
                if(head.compare_exchange_weak(old_head, new_node, std::memory_order_release, std::memory_order_relaxed)) {
                    if(tail.compare_exchange_weak(old_tail, head, std::memory_order_release, std::memory_order_relaxed))
                        break;
                }
            }
            else if(!old_head->next) {  // head has no next since there is only one entry
                if(tail.compare_exchange_weak(old_tail, new_node, std::memory_order_release, std::memory_order_relaxed)) {
                    old_head->next = new_node;
                    break;
                }
            }
            else{  // has both head and tail, we replace tail, update tail->next
                if(tail.compare_exchange_weak(old_tail, new_node, std::memory_order_release, std::memory_order_relaxed)) {
                    old_tail->next = new_node;
                    break;
                }
            }
        }
    }
};

template<class T>
class stack{
public:
    typedef Node<T> MyNode;
    std::atomic<MyNode*> head;

    void push(const T& data)
    {
      MyNode* new_node = new MyNode(data);
 
      // put the current value of head into new_node->next
      new_node->next = head.load(std::memory_order_relaxed);
 
      // now make new_node the new head, but if the head
      // is no longer what's stored in new_node->next
      // (some other thread must have inserted a node just now)
      // then put that new head into new_node->next and try again
      if(true){
      while(!head.compare_exchange_weak(new_node->next, new_node,
                                        std::memory_order_release,
                                        std::memory_order_relaxed))
          ; // the body of the loop is empty
      }
      else{
      MyNode* old_head = head.load(std::memory_order_relaxed);
      do {
          new_node->next = old_head;
       } while(!head.compare_exchange_weak(old_head, new_node,
                                           std::memory_order_release,
                                           std::memory_order_relaxed));
      }
    }
};


// Tell which library to use for the particular command.
using std::cout;
using std::cerr;
using std::endl;

stack<int> testList1;
deque<int> testQueue;

int thread_fun(int my_number) {
    for(int i = 0; i < 100; i++){
        // testList1.push(my_number);
        testQueue.push_right(my_number);
        testList1.push(my_number);
    }
}

int main(int argc, char *argv[]) {
	int thread_num = 1;
    if(argc > 1)
        sscanf(argv[1], "%d", &thread_num);
    std::cout << "Using thread number " << thread_num << std::endl;
	std::vector<std::thread> vec_threads;
	for(int i = 0; i < thread_num; i++) {
		vec_threads.push_back(thread(thread_fun, i));
	}
	// wait
	for(auto &t : vec_threads)
		t.join();
    // print the list
    auto node = testList1.head.load();
    int count = 0;
    while(node) {
        cout << node->data;
        node = node->next;
        count += 1;
    }
    std::cout << std::endl << count << " numbers recorded\n";
    // print the deque
    auto node2 = testQueue.head.load();
    count = 0;
    while(node2) {
        cout << node2->data;
        node2 = node2->next;
        count += 1;
    }
    std::cout << std::endl << count << " numbers recorded\n";
	return 0;
}