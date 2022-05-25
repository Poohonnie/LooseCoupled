#include <iostream>

struct Node
{
    int num{};
    Node *next{};
};

Node *createList(std::initializer_list<int> list)
{
    Node *head = new Node{};
    Node *cur(head);
    for(const auto &list_a : list)
    {
        cur->num = list_a;
        Node *next = new Node{};
        cur->next = next;
        cur = cur->next;
    }
    return head;
}

void dispList(Node *head)
{
    Node *cur(head);
    while(cur->next != nullptr)
    {
        std::cout << cur->num << ' ';
        cur = cur->next;
    }
    std::cout << std::endl;
}

int main()
{
    Node *list = createList({1, 2, 3, 4, 5, 6, 7, 8, 9});
    dispList(list);
    
    
    return 0;
}
