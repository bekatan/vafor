import os
from decouple import config
import openai
import pandas as pd
import time
import random

openai.api_key = config('OPENAI_API_KEY')

# define a retry decorator
def retry_with_exponential_backoff(
    func,
    initial_delay: float = 1,
    exponential_base: float = 2,
    jitter: bool = True,
    max_retries: int = 10,
    errors: tuple = (openai.error.RateLimitError,),
):
    """Retry a function with exponential backoff."""

    def wrapper(*args, **kwargs):
        # Initialize variables
        num_retries = 0
        delay = initial_delay

        # Loop until a successful response or max_retries is hit or an exception is raised
        while True:
            try:
                return func(*args, **kwargs)

            # Retry on specified errors
            except errors as e:
                # Increment retries
                num_retries += 1

                # Check if max retries has been reached
                if num_retries > max_retries:
                    raise Exception(
                        f"Maximum number of retries ({max_retries}) exceeded."
                    )

                # Increment the delay
                delay *= exponential_base * (1 + jitter * random.random())

                # Sleep for the delay
                time.sleep(delay)

            # Raise exceptions for any errors not specified
            except Exception as e:
                raise e

    return wrapper

@retry_with_exponential_backoff
def completions_with_backoff(**kwargs):
    return openai.Completion.create(**kwargs)

df = pd.read_csv('vafor_tests .csv')

objects = df['target'].unique().tolist()

models = ['text-davinci-003','text-curie-001', 'text-babbage-001', 'text-ada-001', ]
# models = ['text-babbage-001', 'text-ada-001', ]

for model in models:
    responses = []
    for sentence, target, *response in df.values:
        prompt = """
    Objects: {}.
    Prompt: Can you bring me a bottle?
    Answer: bottle
    Prompt: I am sleepy.
    Answer: coffee
    Prompt: I want to order pizza.
    Answer: cellphone
    Prompt: {}
    Answer:""".format(', '.join(objects), sentence)
        
        response = completions_with_backoff(model=model, 
                                prompt=prompt, 
                                temperature=0, 
                                top_p=0,
                                max_tokens=100, 
                                )
        
        print(prompt)
        print(response.choices[0].text)

        responses.append(response.choices[0].text.strip())

    df['response'] = responses
    df.to_csv(model + '_panda_responses.csv')