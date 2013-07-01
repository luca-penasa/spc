
template <typename Scalar>
class a
{
    Scalar a;
};

template <typename Scalar>
class b
{
  a<Scalar> other;
};

int main (int argc, char ** argv)
{

    b<float> instance;
    return 1;
}
