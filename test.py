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


completions_with_backoff(model="text-davinci-002", prompt="Once upon a time,")

df = pd.read_csv('vafor_tests .csv')

print(df)

objects = df['target'].unique().tolist()
responces = []

for sentence, target in df.values:
    prompt = """
Which one object can help with the situation in the prompt the most?
Objects: {}.
Prompt: {}
Answer and why:""".format(', '.join(objects), sentence)
    
    response = completions_with_backoff(model="text-davinci-003", 
                             prompt=prompt, 
                             temperature=0, 
                             top_p=0, 
                             )
    
    print(prompt)
    print(response.choices[0].text)

    responces.append(response.choices[0].text)

df['response'] = responces
df.to_csv('vafor_responses.csv')
    # sentence = "I want to eat something."
    # prompt = "Decide which of these is the desired object in the prompt: {}\n".format(', '.join(objects))

    # prompt += """Prompt: "{}"
    # Desired object:""".format(sentence)

    # response = openai.Completion.create(
    #     model="text-davinci-003",
    #     prompt=prompt,
    #     temperature=0,
    # )
    # print(prompt)
    # print(response.choices[0].text)
    # for i in response.choices:
    #     print(i.text)
